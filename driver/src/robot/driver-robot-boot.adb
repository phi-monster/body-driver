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

      --  Finds how far each channel of the group must move for an eye to see
      --  it, both ways from the amount at which the whole body was first seen
      --  (Motion.Probe_Both_Ways: a way at its end stops where the other one
      --  answered; a channel that answers neither way where every other one
      --  did is dead), then pushes every channel both ways by that much, in
      --  an order no other group shares, for the lock-in to tell the groups
      --  apart; never into a way found at its end.
      procedure Recognize (G : Group_Id; Size : Positive; From : Real) is
         use type Driver.Robot.Motion.Sense;
         Start  : constant Real_Array := Holds_Of (G, Size);
         Amount : Real_Array (1 .. Size) := [others => 0.0];
         Ended  : array (1 .. Size) of Driver.Robot.Motion.Sense_Flags := [others => [others => False]];
         Report : Driver.Robot.Motion.Step_Report;
         Order  : Push_Array (1 .. 2 * Size);
         Count  : Natural := 0;
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
               Ended (C) := P.At_End;
               if P.Seen then
                  Amount (C) := P.Excursion;
               end if;
               for S in Driver.Robot.Motion.Sense loop
                  if P.At_End (S) then
                     Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " channel" & C'Image & " is at its end "
                                      & Way (S) & ": it delivered nothing that way up to "
                                      & Scientific (From * 2.0 ** (P.Levels (S) - 1))
                                      & " reading units, while it answered the other way");
                  end if;
               end loop;
               Driver.Log.Line (Driver.Log.Robot, "boot: group" & G'Image & " channel" & C'Image
                                & (if Amount (C) > 0.0
                                   then " is seen when moved by " & Scientific (Amount (C)) & " reading units"
                                   elsif P.Dead
                                   then " answers neither way up to " & Scientific (From * 2.0 ** (P.Levels (Driver.Robot.Motion.Increasing) - 1))
                                        & " reading units, where every other channel of the body had answered: dead or"
                                        & " disconnected; it is not probed further"
                                   else " moves nothing any eye sees, up to where it stops following"));
            end;
         end loop;
         for C in 1 .. Size loop
            if Amount (C) > 0.0 then
               for S in Driver.Robot.Motion.Sense loop
                  if not Ended (C) (S) then
                     Count := Count + 1;
                     Order (Count) := (Channel => C, Sign => (if S = Driver.Robot.Motion.Increasing then 1.0 else -1.0));
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
                  Half := Real (Natural'Min (M.Eyes (Eye_Id (Eye)).Grid.Width, M.Eyes (Eye_Id (Eye)).Grid.Height)) / 2.0;
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
                  begin
                     while Offset * Per_Unit (P.Channel) <= Half loop
                        declare
                           Pose : Real_Array := Start;
                        begin
                           Pose (P.Channel) := Start (P.Channel) + P.Sign * Offset;
                           Go_To (G, Pose, Report);
                           --  An arm given up while it kept moving (chattering
                           --  against what stops it) gives no keyframe there.
                           if Report.At_Rest then
                              Driver.Robot.Motion.Hold_For_Keyframe (M, A);
                           end if;
                           exit when Report.Outcome /= Driver.Robot.Motion.Reached;
                        end;
                        Offset := 2.0 * Offset;
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
      Arms   : Natural := 0;
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
         Arms := Arm_Count (M);
      end Read_Body;
   begin
      Ok := False;
      Driver.Log.Line (Driver.Log.Robot, "boot: holding still to measure the body at rest");
      Driver.Robot.Motion.Settle (M, Waited);
      Driver.Beats.Within_A_Beat (Estimate'Access);
      Driver.Beats.Within_A_Beat (Read_Count'Access);
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
               Driver.Log.Line (Driver.Log.Robot, "boot: no group takes a command; nothing can be moved to be measured");
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
               Driver.Log.Line (Driver.Log.Robot, "boot: every commandable channel moved together is first seen at "
                                & Scientific (P.Excursion) & " reading units, after" & P.Steps'Image & " doublings");
               for G in 1 .. Count loop
                  if Commandable (G) and then Sizes (G) > 0 then
                     Recognize (Group_Id (G), Sizes (G), P.Excursion);
                  end if;
               end loop;
            end;
         end;
      end;
      Driver.Beats.Within_A_Beat (Estimate'Access);
      Driver.Beats.Within_A_Beat (Read_Body'Access);
      for A in 1 .. Arms loop
         Sweep (Arm_Id (A));
      end loop;
      --  The instrument answers the keyframes' matches a beat or more after
      --  they were asked: the fit waits for every answer.
      Driver.Robot.Motion.Hold_While_Matching (M);
      Driver.Beats.Within_A_Beat (Estimate'Access);
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
