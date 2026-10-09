with Ada.Containers.Vectors;
with Ada.Strings;
with Ada.Strings.Fixed;
with Ada.Text_IO;
with Driver.Beats;
with Driver.Commands;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Log;
with Driver.Robot.Body_File;
with Driver.Robot.Lockin;
with Driver.Robot.Motion;
with Driver.Robot.Regression;

package body Driver.Robot.Boot is

   use type Driver.Robot.Motion.Step_Outcome;
   use type Driver.Observations.Group_Id;

   package Real_IO is new Ada.Text_IO.Float_IO (Real);

   --  A number in the log with its magnitude: probe sizes span many decades.
   function Scientific (X : Real) return String is
      S : String (1 .. 32);
   begin
      Real_IO.Put (S, X, Aft => 2, Exp => 3);
      return Ada.Strings.Fixed.Trim (S, Ada.Strings.Both);
   end Scientific;

   function Grew (Before, After, Cells : Integer) return Boolean is
     (After > Before and then Cells >= After
      and then Driver.Robot.Regression.Count_Significant
                 (After - Before, Cells - Before, Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z)));

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
      procedure Push_Both_Ways (G : Group_Id; Size : Positive; Factor : Real; Moved : out Boolean) is
         Start  : constant Real_Array := Holds_Of (G, Size);
         Amount : Real_Array (1 .. Size) := [others => 0.0];
         Report : Driver.Robot.Motion.Step_Report;
         Order  : Push_Array (1 .. 2 * Size);
         Count  : Natural := 0;
      begin
         Moved := False;
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
               --  Each push from a body at rest, its pictures settled: what the eyes see move is the push's own, and
               --  not the tail of the push before it, which the lock-in would credit to this one (Lockin.Measure
               --  leaves out a push that began from a picture still changing).
               Driver.Robot.Motion.Settle (M, Waited);
               Go_To (G, Away, Report);
               Moved := Moved or else Report.Outcome = Driver.Robot.Motion.Reached;
               Driver.Robot.Motion.Settle (M, Waited);
               Go_To (G, Start, Report);
            end;
         end loop;
         Driver.Robot.Motion.Settle (M, Waited);
      end Push_Both_Ways;

      --  Finds how far each channel of the group must move for an eye to see
      --  it, both ways from the amount at which the whole body was first seen
      --  (Motion.Probe_Both_Ways: a way at its end stops where the other one
      --  answered; a channel is pushed on until an eye sees it or its own
      --  reading ends, whatever the other channels needed, and is dead only
      --  when its reading followed no level either way), then pushes every
      --  channel both ways by that much (Push_Both_Ways).
      procedure Recognize (G : Group_Id; Size : Positive; From : Real) is
         Amount : Real_Array (1 .. Size) := [others => 0.0];
         function Way (S : Driver.Robot.Motion.Sense) return String is
           (if S = Driver.Robot.Motion.Increasing then "upwards" else "downwards");
      begin
         for C in 1 .. Size loop
            declare
               Ref : constant Driver.Robot.Motion.Channel_Ref := (Group => G, Channel => C);
               P   : Driver.Robot.Motion.Two_Way_Report;
            begin
               Driver.Robot.Motion.Probe_Both_Ways (M, Ref, From, P);
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
                                        & " reading units, at every level it was asked either way: dead or"
                                        & " disconnected for this boot; it is left alone"
                                   elsif P.Blind
                                   then " moves nothing any eye sees, and its reading's noise is not measured, so how"
                                        & " its reading followed is not known"
                                   else " moves nothing any eye sees, up to where it stops following"));
            end;
         end loop;
         declare
            Moved : Boolean;
         begin
            Push_Both_Ways (G, Size, 1.0, Moved);
         end;
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
               --  Drawn in a held beat: the arms swept at once (Driver.Beats.At_Once)
               --  share the boot's one generator, and their windows never overlap.
               procedure Draw_Order is
               begin
                  Shuffle (Order (1 .. Count), Rng);
               end Draw_Order;
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
               Driver.Beats.Within_A_Beat (Draw_Order'Access);
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
            declare
               Waited_Here : Natural;   --  each arm's own: arms are swept at once
            begin
               Driver.Robot.Motion.Settle (M, Waited_Here);
            end;
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

      --  Every arm of the current estimate is swept with the eye it carries,
      --  every such arm at once (Driver.Beats.At_Once, an arm a lane: their
      --  sweeps move their own groups, and each step waits for the whole body
      --  to be still): the body is read again before each round of sweeps and
      --  after the fit's estimate, since later evidence can change a role or a
      --  mount (an eye can decide by a later estimate that it rides on a
      --  group). An arm is known by its group, since a new arm can renumber the
      --  others, and is swept again when it carries another eye than the one it
      --  was swept with; an arm that carries none has nothing to sweep. The fit
      --  is made when no arm is left.
      procedure Sweep_Every_Arm is
         type Arm_Eye is record
            Group : Group_Id := 1;
            Eye   : Eye_Id := 1;
            Arm   : Arm_Id := 1;
         end record;
         package Pair_Lists is new Ada.Containers.Vectors (Positive, Arm_Eye);
         Swept   : Pair_Lists.Vector;
         Unswept : Pair_Lists.Vector;

         use type Ada.Containers.Count_Type;
         use type Driver.Observations.Camera_Id;

         function Same_Pair (A, B : Arm_Eye) return Boolean is (A.Group = B.Group and then A.Eye = B.Eye);

         procedure Find_Unswept is
         begin
            Unswept.Clear;
            for A in 1 .. Arm_Count (M) loop
               declare
                  Found : Boolean := False;
                  Pair  : Arm_Eye;
               begin
                  for E in 1 .. Eye_Count (M) loop
                     --  The eye the sweep moves: the last one the arm carries.
                     if Eye_Mount (M, Eye_Id (E)).Kind = Arm_Carried
                       and then Eye_Mount (M, Eye_Id (E)).Arm = Arm_Id (A)
                     then
                        Found := True;
                        Pair := (Group => Arm_Group (M, Arm_Id (A)), Eye => Eye_Id (E), Arm => Arm_Id (A));
                     end if;
                  end loop;
                  if Found and then not (for some S of Swept => Same_Pair (S, Pair)) then
                     Unswept.Append (Pair);
                  end if;
               end;
            end loop;
         end Find_Unswept;

         procedure Sweep_Lane (Lane : Positive) is
         begin
            Sweep (Unswept (Lane).Arm);
         end Sweep_Lane;

         procedure Sweep_At_Once is new Driver.Beats.At_Once (Sweep_Lane);
      begin
         loop
            Driver.Beats.Within_A_Beat (Find_Unswept'Access);
            if not Unswept.Is_Empty then
               for P of Unswept loop
                  Driver.Log.Line (Driver.Log.Robot, "boot: sweeping arm" & P.Arm'Image & " (group" & P.Group'Image
                                   & ") with eye" & P.Eye'Image
                                   & (if Unswept.Length > 1 then ", the" & Unswept.Length'Image & " arms at once" else ""));
               end loop;
               if Unswept.Length = 1 then
                  Sweep (Unswept.First_Element.Arm);
               else
                  Sweep_At_Once (Positive (Unswept.Length));
               end if;
               for P of Unswept loop
                  Swept.Append (P);
               end loop;
            else
               --  The instrument answers the keyframes' matches a beat or more
               --  after they were asked: the fit waits for every answer.
               Driver.Robot.Motion.Hold_While_Matching (M);
               Driver.Beats.Within_A_Beat (Estimate'Access);
               Driver.Beats.Within_A_Beat (Find_Unswept'Access);
               exit when Unswept.Is_Empty;
               for P of Unswept loop
                  Driver.Log.Line (Driver.Log.Robot, "boot: the estimate after the sweeps lists arm" & P.Arm'Image
                                   & " (group" & P.Group'Image & ") with eye" & P.Eye'Image & ", not swept yet");
               end loop;
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
                     Driver.Robot.Motion.Probe_Together (M, Refs, 1.0, 0.0, P);
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
                     --  A push at the smallest amount some eye saw is little evidence,
                     --  and an eye that is undecided, or that shows a patch, has not
                     --  answered: an eye on a turning joint sees its view move by
                     --  different amounts in different cells (by depth, by perspective),
                     --  so at that amount only the cells that move most respond and the
                     --  whole picture reads as a patch; pushed harder more cells
                     --  respond, where a patch that is one does not grow. A group some
                     --  such eye is open about is pushed again, at twice the amounts of
                     --  its last round, until every eye has answered (the whole picture
                     --  moves, or nothing does), or a round leaves the cells each open
                     --  eye found responding no more than chance explains (not
                     --  significantly more, at Z and the per-cell false alarm, among
                     --  the cells that did not respond before) and the cells that do not
                     --  respond would show no motion together once the push is doubled
                     --  (Resting_Motion: a far part of the view that moves by less than any
                     --  cell can tell shows in no cell and, a little, in them all, and a
                     --  push of twice the size makes four times that), or a push moves
                     --  nothing further than the one before (the channels are at their
                     --  ends).
                     declare
                        Eyes : Natural := 0;
                        procedure Read_Eyes is
                        begin
                           Eyes := Eye_Count (M);
                        end Read_Eyes;
                     begin
                        Driver.Beats.Within_A_Beat (Read_Eyes'Access);
                        declare
                           --  The cells each open eye found responding after the group's
                           --  last push; -1 for an eye that was not open then.
                           type Count_Grid is array (1 .. Count, 1 .. Eyes) of Integer;
                           Last   : Count_Grid := [others => [others => -1]];
                           Now    : Count_Grid;
                           Cells  : Count_Grid;   --  the cells it was reached over
                           Shows  : array (1 .. Count, 1 .. Eyes) of Eye_Response;
                           Rests  : array (1 .. Count, 1 .. Eyes) of Real;   --  the rest's motion together, in sigmas
                           Factor : array (1 .. Count) of Real := [others => 1.0];
                           Again  : array (1 .. Count) of Boolean;
                           Spent  : array (1 .. Count) of Boolean := [others => False];   --  pushed as far as it goes
                           --  What a round that doubles the push makes of the motion the cells that did not respond
                           --  show together: their energies grow with the square of the push, and so does the sum.
                           Doubled : constant Real := 2.0 ** 2;

                           procedure Read_Open is
                           begin
                              for G in 1 .. Count loop
                                 Again (G) := False;
                                 for E in 1 .. Eyes loop
                                    Now (G, E) := -1;
                                    if Commandable (G)
                                      and then Response (M, Group_Id (G), Eye_Id (E)) in Undecided | Patch
                                    then
                                       Now (G, E) := Responding (M, Group_Id (G), Eye_Id (E));
                                       Cells (G, E) := Textured_Cells (M, Group_Id (G), Eye_Id (E));
                                       Shows (G, E) := Response (M, Group_Id (G), Eye_Id (E));
                                       Rests (G, E) := Resting_Motion (M, Group_Id (G), Eye_Id (E));
                                       Again (G) := not Spent (G)
                                         and then (Again (G) or else Last (G, E) < 0
                                                   or else Grew (Last (G, E), Now (G, E), Cells (G, E))
                                                   or else Doubled * Rests (G, E) > Driver.Conventions.Z);
                                    end if;
                                 end loop;
                              end loop;
                           end Read_Open;
                        begin
                           loop
                              Driver.Beats.Within_A_Beat (Read_Open'Access);
                              for G in 1 .. Count loop
                                 for E in 1 .. Eyes loop
                                    if Now (G, E) >= 0 then
                                       Driver.Log.Line
                                         (Driver.Log.Robot, "boot: group" & G'Image & " eye" & E'Image & " is "
                                          & (if Shows (G, E) = Patch then "a patch" else "undecided") & " at"
                                          & Integer'Image (Integer (Factor (G))) & " times the amounts:"
                                          & Now (G, E)'Image & " of" & Cells (G, E)'Image & " cells respond"
                                          & (if Last (G, E) >= 0 then ", " & Integer'Image (Last (G, E))
                                             & " before" else "")
                                          & ", the rest move together by " & Driver.Log.Image (Rests (G, E), 2)
                                          & " sigmas");
                                    end if;
                                 end loop;
                              end loop;
                              exit when (for all G in Again'Range => not Again (G));
                              for G in Again'Range loop
                                 if Again (G) then
                                    Factor (G) := 2.0 * Factor (G);
                                    Driver.Log.Line
                                      (Driver.Log.Robot, "boot: group" & G'Image
                                       & " leaves an eye undecided or showing a patch that may grow;"
                                       & " pushed again at" & Integer'Image (Integer (Factor (G)))
                                       & " times its amounts");
                                    declare
                                       Moved : Boolean;
                                    begin
                                       Push_Both_Ways (Group_Id (G), Sizes (G), Factor (G), Moved);
                                       if not Moved then
                                          Spent (G) := True;
                                          Driver.Log.Line
                                            (Driver.Log.Robot, "boot: group" & G'Image
                                             & " moved no further when pushed at"
                                             & Integer'Image (Integer (Factor (G)))
                                             & " times its amounts: its channels are at their ends; not pushed again");
                                       end if;
                                    end;
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
