with Ada.Containers.Vectors;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Ada.Unchecked_Deallocation;
with Driver.Conventions;
with Driver.Log;
with Driver.Robot.Channels;
with Driver.Robot.Kinematics;
with Driver.Alignment;
with Driver.Pixels;
with Driver.Robot.Hand.Follow;
with Driver.Robot.Hand.Frames;
with Driver.Robot.Hand.Lobes;
with Driver.Robot.Hand.Lowering;
with Driver.Robot.Hand.Pressing;
with Driver.Robot.Hand.Presses;
with Driver.Robot.Hand.Shape;
with Driver.Robot.Hand.Slide;
with Driver.Robot.Hand.Sweep;
with Driver.Robot.Hand.Tips;
with Driver.Robot.Hand.Views;
with Driver.Robot.Lockin;
with Driver.Robot.Steps;
with Driver.Robot.Stillness;

package body Driver.Robot.Hand is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use type Driver.Observations.Group_Id;
   use type Driver.Observations.Camera_Id;
   use type Driver.Robot.Hand.Sweep.Progress;
   use type Driver.Robot.Hand.Lowering.Verdict;

   package Sweeps renames Driver.Robot.Hand.Sweep;

   --  One closer group as one eye sees it.
   type Pair is record
      Group    : Group_Id;
      Eye      : Eye_Id;
      Arm      : Arm_Id;
      Sweep    : Sweeps.State;
      --  How the eye's picture moved per unit of each closer channel's reading, cell by cell (channel after
      --  channel, the eye's cells in order): each cell's displacement at the beats only that channel moved, times
      --  the channel's step there, summed (Accumulate).
      Motion_U, Motion_V : Real_Vectors.Vector;
      --  How many stops its measured channels had when the hand was last made of them: a stop the decider adds
      --  between two views too far apart to follow makes the hand again, its lobes followed along the stops.
      Stops_Made : Natural := 0;
      --  Whether the eye's place in its arm's tool frame was known when the hand was last made: the tips' lines of
      --  sight and the lobes' shapes rest on it, and a hand made before the arm was fitted is made again once it is.
      --  Made once, at its ends' first measure, the hand kept what the eye's mount was then: u7 (servo seed 3 of the
      --  fake robot, 10-19) made its hand at beat 838, fitted its arm at 1887, and 26 presses later its lobe was
      --  still "its eye's mount was not measured when it was swept", no tip measured.
      Placed_Made : Boolean := False;
   end record;

   package Pair_Vectors is new Ada.Containers.Vectors (Positive, Pair);

   type Patch_Array is array (Opening) of Driver.Robot.Hand.Slide.Patch;

   --  How a lobe's finger closes in its eye's picture: the unit way from where
   --  its tip is open to where it is closed, and how far it goes.
   type Closing_Way is record
      Known : Boolean := False;
      U, V  : Real := 0.0;
      Reach : Real := 0.0;
   end record;

   --  What the eye saw of a lobe at a stop of the sweep between its openings: its tip's line of sight and its look.
   type Stop_Sight is record
      Reading : Real := 0.0;
      Sight   : Hand.Sight;
      Patch   : Driver.Robot.Hand.Slide.Patch;
   end record;

   package Stop_Vectors is new Ada.Containers.Vectors (Positive, Stop_Sight);

   type Lobe_Record is record
      Channel : Natural;   --  of the closer; 0 for the lobe of a tool no closer moves
      Sights  : Sight_Array;
      Stops   : Stop_Vectors.Vector;   --  between the openings, by the channel's reading, ascending
      Patches : Patch_Array;   --  what its tip looked like at each opening, in the views of the sweep
      Way     : Closing_Way;
      Shape   : Driver.Robot.Hand.Shape.Lobe_Shape;   --  its surface as its own eye saw it, up to scale
      Size    : Driver.Robot.Hand.Shape.Lobe_Size;    --  what the presses so far make of it
   end record;

   package Lobe_Record_Vectors is new Ada.Containers.Vectors (Positive, Lobe_Record);

   type Reading_Array is array (Opening) of Driver.Robot.Hand.Views.Reading_Holders.Holder;

   type Hand_Record is record
      Group    : Group_Id'Base;   --  the closer; 0 for the hand an arm's tool is
      Arm      : Arm_Id;
      Eye      : Eye_Id;
      Readings : Reading_Array;
      Lobes    : Lobe_Record_Vectors.Vector;
      Watch    : Driver.Robot.Hand.Presses.Watcher;   --  the arm's presses, from the stream
      Book     : Driver.Robot.Hand.Tips.Book;         --  the presses kept and the tips they measure
      Lowering : Driver.Robot.Hand.Lowering.Track;    --  the descent under way, judged by where the hand went
      Judged   : Natural := 0;                        --  the beat the latest push judged for lowering began
      Stalled  : Natural := 0;                        --  the beat the latest push judged stalled began
      Stalls   : Natural := 0;                        --  how many pushes were judged stalled
      Rests    : Natural := 0;                        --  how many presses the watcher found at a rest, used or not
      Depth    : Estimate;
      Axis     : Direction_Estimate;
      Unsized  : Unbounded_String;                    --  what of its sizes is not measured, and why
   end record;

   package Hand_Vectors is new Ada.Containers.Vectors (Hand_Id, Hand_Record);

   type Hand_Data is record
      Pairs : Pair_Vectors.Vector;
      Found : Hand_Vectors.Vector;
   end record;

   procedure Free is new Ada.Unchecked_Deallocation (Hand_Data, Hand_Data_Access);

   --  A picture's luma, off the stack.
   type Luma_Access is access Real_Array;
   procedure Free is new Ada.Unchecked_Deallocation (Real_Array, Luma_Access);

   overriding procedure Finalize (H : in out Hands) is
   begin
      Free (H.Data);
   end Finalize;

   function Image (P : Driver.Images.Pixel) return String is
     ("(" & Driver.Log.Image (P.U, 1) & ", " & Driver.Log.Image (P.V, 1) & ")");

   --  Every other group's readings in group order, and their noises in the
   --  same order: what must not change while one view of a closer is taken.

   function Rest_Of (O : Observation; G : Group_Id) return Real_Array is
      Total : Natural := 0;
   begin
      for Other in O.Readings.First_Index .. O.Readings.Last_Index loop
         if Other /= G then
            Total := Total + O.Readings.Element (Other)'Length;
         end if;
      end loop;
      declare
         R : Real_Array (1 .. Total);
         K : Natural := 0;
      begin
         for Other in O.Readings.First_Index .. O.Readings.Last_Index loop
            if Other /= G then
               for V of O.Readings (Other) loop
                  K := K + 1;
                  R (K) := V;
               end loop;
            end if;
         end loop;
         return R;
      end;
   end Rest_Of;

   function Closer_Sigma (M : Model; G : Group_Id) return Real_Array is
      --  The standard deviation of each of the closer's readings as the body's one test of motion sees them
      --  (Channels.Visible): a change below the step an eye can see (Visible_Step) is none, so the sigma of a
      --  reading whose change of two of them is significant from that step on, or the reading's own noise where
      --  that is larger or no eye has told a step. Its settings, its views' readings and its openings are told
      --  apart by it. In a simulator a reading repeats to its last digit and its noise is none, and any digit told
      --  them apart (A74: arm 2's raises jostled its closer by what no eye sees, its poses went to a setting of
      --  their own, the setting of its ends never had them, and no hand was made of it).
      N      : constant Natural := Group_Size (M, G);
      Result : Real_Array (1 .. N);
   begin
      for C in 1 .. N loop
         declare
            Step : constant Estimate := Visible_Step (M, G, C);
         begin
            Result (C) := Reading_Noise (M, G, C);
            if Known (Step) then
               Result (C) := Real'Max (Result (C), Step.Value / (Driver.Conventions.Z * Sqrt (2.0)));
            end if;
            --  Two landings of one asked setting differ by up to twice what a free push of the group lands off its
            --  target by (Steps.Free_Landing: a servo stops anywhere within its dead band): readings no nearer are one
            --  setting as far as sending the closer there can tell. h09 and h15 (servo seed 3, 10-17): the closer sent
            --  back to its end at 601 read 605, four ticks off where the eyes' least step is 1.2, a setting of its own,
            --  and the poses gathered there were never the end's; no hand was made. Twice the least ask the body
            --  executed did not cover it: s21 (the same body), sent back to 605, read 593 and then 617, to 677 read
            --  669, three ticks of four units off either way, and again no hand was made.
            Result (C) := Real'Max
              (Result (C), 2.0 * Driver.Robot.Steps.Free_Landing (M, G) / (Driver.Conventions.Z * Sqrt (2.0)));
         end;
      end loop;
      return Result;
   end Closer_Sigma;

   function Rest_Moved (M : Model; G : Group_Id; Before, After : Real_Array) return Boolean is
      --  Every other group's readings, as Rest_Of lays them out, through the
      --  body's own one test of motion: a channel an eye watches moves only
      --  by a step that eye can see. Laid out otherwise, something came or
      --  went: that is a change.
      K : Natural := 0;
   begin
      if Before'Length /= After'Length then
         return True;
      end if;
      for Other in 1 .. Group_Id'Base (Group_Count (M)) loop
         if Other /= G then
            declare
               N : constant Natural := Group_Size (M, Other);
            begin
               if K + N > Before'Length then
                  return True;
               end if;
               declare
                  D : constant Real_Array (1 .. N) :=
                    [for C in 1 .. N => After (After'First + K + C - 1) - Before (Before'First + K + C - 1)];
               begin
                  if Driver.Robot.Channels.Visible (M, Other, D) then
                     return True;
                  end if;
               end;
               K := K + N;
            end;
         end if;
      end loop;
      return K /= Before'Length;
   end Rest_Moved;

   function Eye_Moved (M : Model; G : Group_Id; E : Eye_Id; Before, After : Real_Array) return Boolean is
      --  Every other group's readings, as Rest_Of lays them out: a group that moves the whole of the eye's picture
      --  (it carries the eye) moved by a step the eye's cells tell (Lockin.Shows_Step). Laid out otherwise,
      --  something came or went: that is a change.
      K : Natural := 0;
   begin
      if Before'Length /= After'Length then
         return True;
      end if;
      for Other in 1 .. Group_Id'Base (Group_Count (M)) loop
         if Other /= G then
            declare
               N : constant Natural := Group_Size (M, Other);
            begin
               if K + N > Before'Length then
                  return True;
               end if;
               if Response (M, Other, E) = Whole then
                  declare
                     D : constant Real_Array (1 .. N) :=
                       [for C in 1 .. N => After (After'First + K + C - 1) - Before (Before'First + K + C - 1)];
                  begin
                     if Driver.Robot.Lockin.Shows_Step (M, E, Other, D) then
                        return True;
                     end if;
                  end;
               end if;
               K := K + N;
            end;
         end if;
      end loop;
      return K /= Before'Length;
   end Eye_Moved;

   function All_Read (O : Observation) return Boolean is
     (for all G in O.Readings.First_Index .. O.Readings.Last_Index => Driver.Observations.Has_Reading (O, G));

   function Has_Pair (D : Hand_Data; G : Group_Id; E : Eye_Id) return Boolean is
     (for some P of D.Pairs => P.Group = G and then P.Eye = E);

   procedure Keep (D : in out Hand_Data; Made : in out Hand_Record);
   --  Keeps the hand made (declared ahead: Find_Pairs makes the hand an arm's tool is).

   procedure Find_Pairs (D : in out Hand_Data; M : Model; O : Observation);
   --  Every closer group with the eyes on its own arm, once the body has
   --  measured them; by the body's roles as they are now, which the boot
   --  re-reads: a pair whose group is no longer a closer, or no longer of that
   --  arm, goes, and so does a hand found from it. An eye that rides on
   --  another arm or on nothing is not a pair: what the eye shows of the robot
   --  itself as the arm moves (Driver.Robot.Hand.Selfsight) is the arm-carried
   --  eye's.

   function Still_A_Pair (P : Pair; M : Model) return Boolean is
     (Role (M, P.Group) = Closer and then Closer_Arm (M, P.Group) = P.Arm
      and then Eye_Mount (M, P.Eye).Kind = Arm_Carried and then Eye_Mount (M, P.Eye).Arm = P.Arm);

   function Has_A_Closer (M : Model; A : Arm_Id) return Boolean is
     (for some G in 1 .. Group_Id'Base (Group_Count (M)) => Role (M, G) = Closer and then Closer_Arm (M, G) = A);

   --  A hand of a closer stands while the body takes its group for a closer of its arm; the hand an arm's tool
   --  is (no closer) stands while the arm has no closer, its eye still rides on it and the arm is fitted (the
   --  tool frame its presses are made in).
   function Still_A_Hand (R : Hand_Record; M : Model) return Boolean is
     (if R.Group > 0 then Role (M, R.Group) = Closer and then Closer_Arm (M, R.Group) = R.Arm
      else Natural (R.Arm) <= Arm_Count (M) and then not Has_A_Closer (M, R.Arm)
           and then Natural (R.Eye) <= Eye_Count (M) and then Eye_Mount (M, R.Eye).Kind = Arm_Carried
           and then Eye_Mount (M, R.Eye).Arm = R.Arm and then Driver.Robot.Kinematics.Fitted (M, R.Arm));

   procedure Find_Pairs (D : in out Hand_Data; M : Model; O : Observation) is
   begin
      declare
         I : Positive := 1;
      begin
         while I <= Natural (D.Pairs.Length) loop
            if Still_A_Pair (D.Pairs (I), M) then
               I := I + 1;
            else
               declare
                  --  Why, for the log: what the roles say of the group and of the eye now, and every eye's verdict on
                  --  the group (s32, s36, s37, s38 on 10-17 each lost their closer at an estimate, and the lines said
                  --  nothing of what had changed).
                  G    : constant Group_Id := D.Pairs (I).Group;
                  Says : Ada.Strings.Unbounded.Unbounded_String;
               begin
                  for E in 1 .. Eye_Id'Base (Eye_Count (M)) loop
                     Ada.Strings.Unbounded.Append (Says, " eye" & E'Image & " " & Response (M, G, E)'Image);
                  end loop;
                  Driver.Log.Line (Driver.Log.Robot, "hand: group" & G'Image & " is no longer a closer"
                                   & " of arm" & D.Pairs (I).Arm'Image & " watched in eye" & D.Pairs (I).Eye'Image
                                   & "; its sweep there is dropped: the group is " & Role (M, G)'Image
                                   & (if Role (M, G) = Closer then " of arm" & Closer_Arm (M, G)'Image else "")
                                   & ", eye" & D.Pairs (I).Eye'Image & " " & Eye_Mount (M, D.Pairs (I).Eye).Kind'Image
                                   & (if Eye_Mount (M, D.Pairs (I).Eye).Kind = Arm_Carried
                                      then " on arm" & Eye_Mount (M, D.Pairs (I).Eye).Arm'Image else "")
                                   & "; the eyes on the group:" & Ada.Strings.Unbounded.To_String (Says));
               end;
               D.Pairs.Delete (I);
            end if;
         end loop;
      end;
      declare
         I : Hand_Id := 1;
      begin
         while I <= D.Found.Last_Index loop
            if Still_A_Hand (D.Found (I), M) then
               I := I + 1;
            else
               Driver.Log.Line (Driver.Log.Robot,
                                (if D.Found (I).Group > 0
                                 then "hand: group" & D.Found (I).Group'Image & " is no longer a closer of arm"
                                      & D.Found (I).Arm'Image & "; the hand found from it is dropped"
                                 else "hand: arm" & D.Found (I).Arm'Image & " has a closer now, or lost its eye or its fit;"
                                      & " the hand its tool was is dropped"));
               D.Found.Delete (I);
            end if;
         end loop;
      end;
      --  An arm that carries an eye and is fitted, with no closer group of its own, has the hand its tool is: one
      --  lobe that no channel moves and no eye has a line of sight to, able to press and push; its tip is where
      --  its presses stop (Tips, the free model of Touch). Made once the arm is fitted (the tool frame the presses
      --  are made in), kept while Still_A_Hand.
      for A in 1 .. Arm_Id'Base (Arm_Count (M)) loop
         if not Has_A_Closer (M, A) and then Driver.Robot.Kinematics.Fitted (M, A)
           and then not (for some R of D.Found => R.Group = 0 and then R.Arm = A)
         then
            for E in 1 .. Eye_Id'Base (Eye_Count (M)) loop
               if Eye_Mount (M, E).Kind = Arm_Carried and then Eye_Mount (M, E).Arm = A then
                  declare
                     Made : Hand_Record := (Group => 0, Arm => A, Eye => E, others => <>);
                     None : constant Real_Array (1 .. 0) := [];
                  begin
                     Made.Readings := [Open         => Driver.Robot.Hand.Views.Reading_Holders.To_Holder (None),
                                       Closed_Empty => Driver.Robot.Hand.Views.Reading_Holders.To_Holder (None)];
                     Made.Lobes.Append
                       (Lobe_Record'(Channel => 0,
                                     Sights  => <>,
                                     Stops   => <>,
                                     Patches => [others => Driver.Robot.Hand.Slide.Empty],
                                     Way     => <>,
                                     Shape   => Driver.Robot.Hand.Shape.Unfitted ("no closer moves it: its tip is where its presses stop"),
                                     Size    => <>));
                     Driver.Log.Line (Driver.Log.Robot, "hand: arm" & A'Image & " carries eye" & E'Image
                                      & " and no closer: its tool is a hand of one lobe, pressed to find its tip");
                     Keep (D, Made);
                     exit;
                  end;
               end if;
            end loop;
         end if;
      end loop;
      for G in 1 .. Group_Id'Base (Group_Count (M)) loop
         if Role (M, G) = Closer and then Closer_Arm (M, G) > 0 and then Driver.Observations.Has_Reading (O, G) then
            for E in 1 .. Eye_Id'Base (Eye_Count (M)) loop
               declare
                  Arm      : constant Arm_Id := Closer_Arm (M, G);
                  Mount_Of : constant Mount := Eye_Mount (M, E);
                  Own      : constant Boolean := Mount_Of.Kind = Arm_Carried and then Mount_Of.Arm = Arm;
               begin
                  if Own and then Driver.Observations.Has_Image (O, E) and then not Has_Pair (D, G, E) then
                     declare
                        Channels : constant Positive := O.Readings.Element (G)'Length;
                        Noise    : constant Real_Array (1 .. Channels) := Closer_Sigma (M, G);
                     begin
                        D.Pairs.Append
                          (Pair'(Group => G,
                            Eye   => E,
                            Arm   => Arm,
                            Sweep => Sweeps.Start (Driver.Images.Width (O.Images (E)), Driver.Images.Height (O.Images (E)),
                                                   Channels, Noise),
                            others => <>));
                        Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & G'Image & " on arm" & Arm'Image
                                         & " is watched in eye" & E'Image & ", its own");
                     end;
                  end if;
               end;
            end loop;
         end if;
      end loop;
   end Find_Pairs;

   procedure Accumulate (P : in out Pair; M : Model);
   --  Adds the latest beat's picture motion to the pair's tally when one channel of the closer moved, by a step the
   --  body's one test of motion sees: each resolved cell's displacement in the picture that shows the step (the eye's
   --  image lag after it), times the step; and when no other commandable group's step at that beat moves this eye's
   --  picture by what its cells tell (Lockin.Shows_Step): a beat whose picture another group moved shows that motion
   --  as well, and the world moves past a closer carried with its eye (A88's replay with every such beat tallied:
   --  28,242 of the lobes' pixels went with the closer's motion and 34,938 against it, and the ends the robot's look
   --  had given right were turned wrong). Whether another group moved by the body's one test of motion is not the
   --  question: the arm holding the closer creeps by a millionth of a radian a beat, which that test sees and no cell
   --  of the eye does (A88's hand 1 tallied nothing so, and its wrong ends were not turned).

   procedure Accumulate (P : in out Pair; M : Model) is
      S    : Eye_Stream renames M.Eyes (P.Eye);
      N    : constant Natural := Cells (S.Grid);
      B    : constant Integer := M.Beats - 1;
      R    : constant Integer := B - Integer'Max (0, Image_Lag (M, P.Eye));
      Size : constant Natural := Group_Size (M, P.Group);
   begin
      if N = 0 or else Size = 0 or else R < 1 or else Natural (S.Du.Length) < (B + 1) * N
        or else Natural (S.Resolved.Length) < (B + 1) * N
        or else not Driver.Robot.Channels.Has_Reading (M, P.Group, R)
        or else not Driver.Robot.Channels.Has_Reading (M, P.Group, R - 1)
        or else not Driver.Robot.Channels.Moving (M, P.Group, R)
      then
         return;
      end if;
      if Natural (P.Motion_U.Length) /= Size * N then
         P.Motion_U.Clear;
         P.Motion_V.Clear;
         P.Motion_U.Append (0.0, Ada.Containers.Count_Type (Size * N));
         P.Motion_V.Append (0.0, Ada.Containers.Count_Type (Size * N));
      end if;
      declare
         Moved : Natural := 0;
         Which : Positive := 1;
      begin
         for C in 1 .. Size loop
            declare
               Alone : Real_Array (1 .. Size) := [others => 0.0];
            begin
               Alone (C) := Driver.Robot.Channels.Change (M, P.Group, R, C);
               if Driver.Robot.Channels.Visible (M, P.Group, Alone) then
                  Moved := Moved + 1;
                  Which := C;
               end if;
            end;
         end loop;
         if Moved /= 1 then
            return;
         end if;
         for G in 1 .. Group_Id'Base (Group_Count (M)) loop
            if G /= P.Group and then Is_Commandable (M, G) and then Driver.Robot.Channels.Has_Reading (M, G, R)
              and then Driver.Robot.Channels.Has_Reading (M, G, R - 1)
              and then Driver.Robot.Lockin.Shows_Step
                (M, P.Eye, G, [for C in 1 .. Group_Size (M, G) => Driver.Robot.Channels.Change (M, G, R, C)])
            then
               return;
            end if;
         end loop;
         declare
            Step : constant Real := Driver.Robot.Channels.Change (M, P.Group, R, Which);
            Base : constant Natural := (Which - 1) * N;
         begin
            for Cell in 0 .. N - 1 loop
               if S.Resolved (B * N + Cell) then
                  P.Motion_U.Replace_Element (Base + Cell, P.Motion_U (Base + Cell) + Step * S.Du (B * N + Cell));
                  P.Motion_V.Replace_Element (Base + Cell, P.Motion_V (Base + Cell) + Step * S.Dv (B * N + Cell));
               end if;
            end loop;
         end;
      end;
   end Accumulate;

   procedure Feed (P : in out Pair; M : Model; O : Observation);

   procedure Feed (P : in out Pair; M : Model; O : Observation) is
      Complete : constant Boolean := All_Read (O) and then Driver.Observations.Has_Image (O, P.Eye);
      function Moved (Before, After : Real_Array) return Boolean is (Rest_Moved (M, P.Group, Before, After));
      function Carried (Before, After : Real_Array) return Boolean is (Eye_Moved (M, P.Group, P.Eye, Before, After));

      --  The tally at a pixel: its cell's (Accumulate). Cells are numbered row by row from the top-left one, a cell
      --  covering the columns from its number times the width over the columns on (Flow.Bounds).
      procedure Motion (Channel : Positive; Column, Row : Natural; U, V : out Real) is
         G : constant Cell_Grid := M.Eyes (P.Eye).Grid;
      begin
         U := 0.0;
         V := 0.0;
         if G.Width > 0 and then G.Height > 0 and then Column < G.Width and then Row < G.Height then
            declare
               K : constant Natural :=
                 (Channel - 1) * Cells (G) + ((Row + 1) * G.Rows - 1) / G.Height * G.Columns
                 + ((Column + 1) * G.Columns - 1) / G.Width;
            begin
               if K < Natural (P.Motion_U.Length) then
                  U := P.Motion_U (K);
                  V := P.Motion_V (K);
               end if;
            end;
         end if;
      end Motion;
   begin
      Accumulate (P, M);
      Sweeps.Set_Closer_Noise (P.Sweep, Closer_Sigma (M, P.Group));
      --  A view of the pair's eye needs that eye's picture to have stopped
      --  changing (the one stop rule, Driver.Robot.Stillness): its validity
      --  is its eye's, not the far arm's. Whether the rest of the body moved
      --  between two of its readings is the body's one test of motion
      --  (Rest_Moved), not the views' own: by its readings' noise alone, A12's
      --  arms, creeping 1.2e-12 rad a beat for an hour, moved at every beat,
      --  every beat started the view again, and none ever had two frames.
      Sweeps.Observe (P.Sweep, O, Still => Driver.Robot.Stillness.Eye_Settled (M, P.Eye) and then Complete,
                      Closer => O.Readings (P.Group), Rest => Rest_Of (O, P.Group), Image => O.Images (P.Eye),
                      Rest_Moved => Moved'Access, Eye_Moved => Carried'Access, Motion => Motion'Access);
   end Feed;

   function Sizes_Text (R : Hand_Record) return String;
   --  The hand's sizes as measured so far, for the log.

   function Sizes_Text (R : Hand_Record) return String is
      Text : Unbounded_String;
      function Image (E : Estimate) return String is
        (if Known (E) then Driver.Log.Image (E.Value, 4) & " +- " & Driver.Log.Image (E.Sigma, 4) else "unmeasured");
   begin
      for L in R.Lobes.First_Index .. R.Lobes.Last_Index loop
         Append (Text, "lobe" & L'Image & " width " & Image (R.Lobes (L).Size.Width)
                 & ", thickness " & Image (R.Lobes (L).Size.Thickness)
                 & ", face " & Image (R.Lobes (L).Size.Face) & " ahead of its tip"
                 & (if Driver.Robot.Hand.Shape.Fitted (R.Lobes (L).Shape)
                    then " (" & Natural'Image (Driver.Robot.Hand.Shape.Kept (R.Lobes (L).Shape)) & " points, scatter "
                         & Driver.Log.Image (Driver.Robot.Hand.Shape.Scatter (R.Lobes (L).Shape), 3) & ")"
                    else "")
                 & "; ");
      end loop;
      Append (Text, "depth " & Image (R.Depth));
      if Length (R.Unsized) > 0 then
         Append (Text, "; " & To_String (R.Unsized));
      end if;
      return To_String (Text);
   end Sizes_Text;

   procedure Size_Up (R : in out Hand_Record; Id : Hand_Id);
   --  The hand's sizes again, from its lobes' shapes and the tips its
   --  presses measure now.

   procedure Size_Up (R : in out Hand_Record; Id : Hand_Id) is
      package Shapes renames Driver.Robot.Hand.Shape;
      N      : constant Natural := Natural (R.Lobes.Length);
      Fits   : Shapes.Lobe_Shape_Array (1 .. N);
      Tipped : Shapes.Tip_Array (1 .. N, Opening);
      Unmeasured : Point_Estimate;
   begin
      if N = 0 then
         return;
      end if;
      --  A size rests on tips two presses have landed on: a provisional tip
      --  is a bound, and what is taken from it would be taken for measured.
      for L in 1 .. N loop
         Fits (L) := R.Lobes (L).Shape;
         for O in Opening loop
            Tipped (L, O) := (if Driver.Robot.Hand.Tips.Confirmed (R.Book, L, O)
                              then Driver.Robot.Hand.Tips.Tip (R.Book, L, O) else Unmeasured);
         end loop;
      end loop;
      declare
         Sized : constant Shapes.Hand_Size := Shapes.Measure (Fits, Tipped);
      begin
         for L in 1 .. N loop
            R.Lobes (L).Size := Sized.Sizes (L);
         end loop;
         R.Depth := Sized.Depth;
         R.Axis := Sized.Axis;
         R.Unsized := Sized.Why;
      end;
      Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & " sizes: " & Sizes_Text (R));
   end Size_Up;

   --  The hand found from a closer group, among the hands. A hand measured
   --  again keeps its presses; they are given to the new lobes by number, or
   --  forgotten when the number changed.

   procedure Keep (D : in out Hand_Data; Made : in out Hand_Record) is
      Table : Driver.Robot.Hand.Tips.Sight_Table (1 .. Natural (Made.Lobes.Length));
   begin
      for L in Table'Range loop
         for Which in Opening loop
            --  A lobe no channel moves and no eye sees (a tool's) has no line of sight: its presses go to the book as a
            --  tip anywhere, starting from the tool frame's origin (Touch, the free model). A fixed jaw is seen.
            Table (L) (Which) :=
              (if Made.Lobes (L).Channel = 0 and then not Made.Lobes (L).Sights (Which).Known
               then (Known  => False,
                     Lined  => False,
                     Ray    => (Origin    => (Mean => Zero3, Covariance => [others => [others => 0.0]]),
                                Direction => (Unit_Vector => [0.0, 0.0, 1.0], Sigma => Real'Last)),
                     Spread => 0.0,
                     Travel => 0.0,
                     Pitch  => 0.0,
                     Face   => Corner_Vectors.Empty_Vector)
               else (Known  => Made.Lobes (L).Sights (Which).Known,
                     Lined  => True,
                     Ray    => Made.Lobes (L).Sights (Which).Ray,
                     Spread => Made.Lobes (L).Sights (Which).Spread,
                     Travel => Made.Lobes (L).Way.Reach * Made.Lobes (L).Sights (Which).Pitch,
                     Pitch  => Made.Lobes (L).Sights (Which).Pitch,
                     Face   => Made.Lobes (L).Sights (Which).Face));
         end loop;
      end loop;
      for Id in D.Found.First_Index .. D.Found.Last_Index loop
         if D.Found (Id).Group = Made.Group and then (Made.Group > 0 or else D.Found (Id).Arm = Made.Arm) then
            Made.Watch := D.Found (Id).Watch;
            Made.Book := D.Found (Id).Book;
            Driver.Robot.Hand.Tips.Set_Sights (Made.Book, Table);
            Size_Up (Made, Id);
            D.Found.Replace_Element (Id, Made);
            return;
         end if;
      end loop;
      Driver.Robot.Hand.Tips.Set_Sights (Made.Book, Table);
      Size_Up (Made, D.Found.Last_Index + 1);
      D.Found.Append (Made);
   end Keep;

   procedure Rebuild (D : in out Hand_Data; P : Pair; M : Model);
   --  The hand of a closer group from what its own eye measured: every
   --  measured channel's lobes with their tips at both openings.

   procedure Rebuild (D : in out Hand_Data; P : Pair; M : Model) is
      Made : Hand_Record := (Group => P.Group, Arm => P.Arm, Eye => P.Eye, others => <>);

      function Seen_Tip
        (V      : Driver.Robot.Hand.Views.View;
         Known  : Boolean;
         Px     : Driver.Images.Pixel;
         Region : Driver.Images.Mask;
         Centre : Driver.Images.Pixel) return Sight
      is
      begin
         if not Known then
            return (Known => False, Pixel => Px, Ray => <>, Spread => 0.0, Pitch => 0.0, Face => <>);
         end if;
         declare
            Line  : constant Ray_Estimate := Eye_Ray (M, P.Eye, Px);
            Next  : constant Ray_Estimate := Eye_Ray (M, P.Eye, (U => Px.U + 1.0, V => Px.V));
            --  The angle one pixel spans at the tip.
            Pitch : constant Real :=
              (if Line.Direction.Sigma < Real'Last and then Next.Direction.Sigma < Real'Last
               then Arctan (abs Cross (Line.Direction.Unit_Vector, Next.Direction.Unit_Vector),
                            Line.Direction.Unit_Vector * Next.Direction.Unit_Vector)
               else 0.0);
            Ray   : constant Ray_Estimate := Driver.Robot.Hand.Frames.Into (Eye_In_Tool (M, P.Eye, V.Seen), Line);
            Cap   : constant Driver.Images.Mask := Driver.Robot.Hand.Lobes.Tip_Cap (Region, Centre, Px);
            --  The tip's face: the corners of the tip region's outline, their lines taken into the tool frame as the
            --  tip's is.
            Face  : Corner_Vectors.Vector;
         begin
            for Corner of Driver.Robot.Hand.Lobes.Tip_Outline (Cap) loop
               declare
                  Its : constant Ray_Estimate :=
                    Driver.Robot.Hand.Frames.Into (Eye_In_Tool (M, P.Eye, V.Seen), Eye_Ray (M, P.Eye, Corner));
               begin
                  if Its.Direction.Sigma < Real'Last and then abs Its.Direction.Unit_Vector > 0.0 then
                     Face.Append (Its.Direction.Unit_Vector);
                  end if;
               end;
            end loop;
            --  A sight is known when its line is: the eye's lens and its place in the tool frame are the arm's
            --  fit's, and until the arm is fitted a tip seen in the picture has no line of sight to press along
            --  (the first fake body: its arm's focal length undetermined, its hand swept, and the press's aim
            --  asked the direction of a zero vector).
            return (Known  => Ray.Direction.Sigma < Real'Last and then abs Ray.Direction.Unit_Vector > 0.0,
                    Pixel  => Px,
                    Ray    => Ray,
                    Spread => Pitch * Driver.Robot.Hand.Lobes.Tip_Spread (Cap, Px),
                    Pitch  => Pitch,
                    Face   => Face);
         end;
      end Seen_Tip;

      --  The lobe's shape (Driver.Robot.Hand.Shape) from its pixels followed along the closer's still views, from
      --  the open end through the stops between to the closed end (Driver.Robot.Hand.Follow): each followed point
      --  with where it is at the closed end, and the tip, the point the presses measure, from where the placing put
      --  it at the open end to where it put it at the closed one; their lines of sight taken into the tool frame
      --  through the eye's mount as it was when the open end was seen. A stop where the lobe's tip was not found is
      --  passed over. Where a step on the way is wider than the matcher follows, the views from the open end up to
      --  it are followed and fitted, a lobe followed part of its travel (Shape.Fit's Closed_Is_End): a jaw swung
      --  onto a fixed one is not told from it near the closed end, its tip seen at none of the stops there however
      --  fine the closer stops (a13 on 10-16: 84 pixels between 0.697 and 0.814, thirteen stops between and no tip
      --  at any), and its shape, its width and how deep a thing goes between the jaws are what the open part of
      --  its travel shows.
      function Followed_Shape
        (C : Positive; Lobe_No : Positive; L : Driver.Robot.Hand.Lobes.Lobe;
         Low, High : Driver.Robot.Hand.Views.View; Closed_High : Boolean) return Driver.Robot.Hand.Shape.Lobe_Shape
      is
         package Shapes renames Driver.Robot.Hand.Shape;
         package Follow renames Driver.Robot.Hand.Follow;
         Mount_Then : constant Pose_Estimate := Eye_In_Tool (M, P.Eye, (if Closed_High then Low.Seen else High.Seen));
         Open_Tip   : constant Driver.Images.Pixel := (if Closed_High then L.Tip_Here else L.Tip_There);
         Closed_Tip : constant Driver.Images.Pixel := (if Closed_High then L.Tip_There else L.Tip_Here);
         Open_Mask  : constant Driver.Images.Mask := (if Closed_High then L.Here else L.There);
         Bordered   : constant Boolean := (if Closed_High then L.Bordered_Here else L.Bordered_There);
         Mount      : Pose_Estimate renames Mount_Then;
         Width      : constant Natural := Driver.Images.Width (Open_Mask);
         Height     : constant Natural := Driver.Images.Height (Open_Mask);
         --  The views where the lobe's tip is known, from the open end to the closed one: the ends, and the stops
         --  between them in the order of their readings, turned round when the open end is the high one.
         N          : constant Natural := Sweeps.Stops (P.Sweep, C);
         Order      : array (1 .. N + 2) of Natural := [others => 0];   --  0, the low end; N + 1, the high; else a stop
         Tips       : Follow.Pixel_Array (1 .. N + 2);
         --  Where its tip region is in each (Lobes.Tip_Track): what the steps between views are measured by, and
         --  the follow's windows; the tip pixel, a corner of a flat end, crosses it from view to view.
         Track      : Follow.Pixel_Array (1 .. N + 2);
         Readings   : Real_Array (1 .. N + 2);   --  the channel's reading at each view kept
         Count      : Natural := 0;
         All_Views  : Natural := 0;   --  every view where the tip is seen; Count, those followed
         function Partial return Boolean is (Count < All_Views);

         function Line (Px : Driver.Images.Pixel) return Vec3 is
           (Mount.Pose.Rotation * Eye_Ray (M, P.Eye, Px).Direction.Unit_Vector);

         procedure Take (K : Natural) is
         begin
            if K = 0 then
               Count := Count + 1;
               Order (Count) := 0;
               Tips (Count) := L.Tip_Here;
               Track (Count) := Driver.Robot.Hand.Lobes.Tip_Track (L.Here, L.Centre_Here, L.Tip_Here);
               Readings (Count) := Driver.Robot.Hand.Views.Reading (Low, C);
            elsif K = N + 1 then
               Count := Count + 1;
               Order (Count) := N + 1;
               Tips (Count) := L.Tip_There;
               Track (Count) := Driver.Robot.Hand.Lobes.Tip_Track (L.There, L.Centre_There, L.Tip_There);
               Readings (Count) := Driver.Robot.Hand.Views.Reading (High, C);
            else
               declare
                  At_Stop : constant Sweeps.Stop_Tips := Sweeps.Tips_At_Stop (P.Sweep, C, K);
               begin
                  if Lobe_No in At_Stop'Range and then At_Stop (Lobe_No).Known then
                     Count := Count + 1;
                     Order (Count) := K;
                     Tips (Count) := At_Stop (Lobe_No).Tip;
                     Track (Count) := Driver.Robot.Hand.Lobes.Tip_Track
                       (At_Stop (Lobe_No).Region, At_Stop (Lobe_No).Centre, At_Stop (Lobe_No).Tip);
                     Readings (Count) := Sweeps.Stop_Reading (P.Sweep, C, K);
                  end if;
               end;
            end if;
         end Take;
      begin
         if not (L.Tip_Known_Here and then L.Tip_Known_There) then
            return Shapes.Unfitted ("its tip is not seen at both ends");
         elsif Mount.Position_Covariance (1, 1) = Real'Last then
            return Shapes.Unfitted ("its eye's mount was not measured when it was swept");
         elsif Eye_Ray (M, P.Eye, Open_Tip).Direction.Sigma = Real'Last then
            return Shapes.Unfitted ("its eye's lens was not measured when it was swept");
         end if;
         for J in 0 .. N + 1 loop
            Take (if Closed_High then J else N + 1 - J);
         end loop;
         All_Views := Count;
         declare
            Wide : constant Natural := Follow.Widest_Step (Track (1 .. Count), Width, Height);
         begin
            if Wide = 1 then
               return Shapes.Not_Followed
                 ("its pixels are not followed from the open end: its tip moves"
                  & Natural'Image (Natural (Follow.Tip_Step (Track (1 .. Count), Wide))) & " pixels between the first two"
                  & " of the" & Count'Image & " views where it is seen, from " & Image (Track (Wide)) & " at "
                  & Driver.Log.Image (Readings (Wide), 6) & " to " & Image (Track (Wide + 1)) & " at "
                  & Driver.Log.Image (Readings (Wide + 1), 6) & ", farther than the matcher searches at its finest ("
                  & Natural'Image (Natural (Driver.Alignment.Finest_Radius (Width, Height))) & " pixels)");
            elsif Wide > 1 then
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & P.Group'Image & " channel" & C'Image & " lobe"
                                & Lobe_No'Image & ": followed from the open end to " & Driver.Log.Image (Readings (Wide), 6)
                                & ", where its tip moves" & Natural'Image (Natural (Follow.Tip_Step (Track (1 .. Count), Wide)))
                                & " pixels to the next view, at " & Driver.Log.Image (Readings (Wide + 1), 6)
                                & ": part of its travel");
               Count := Wide;
            end if;
         end;
         declare
            Views : Follow.Pyramid_Array (1 .. Count);
            Luma  : Luma_Access := new Real_Array (1 .. Width * Height);
         begin
            for I in 1 .. Count loop
               declare
                  V : constant Driver.Robot.Hand.Views.View :=
                    (if Order (I) = 0 then Low elsif Order (I) = N + 1 then High
                     else Sweeps.Stop_View (P.Sweep, C, Order (I)));
               begin
                  Driver.Pixels.Means (V.Frames, Luma.all);
                  Views (I) := Driver.Alignment.Pyramid_Of (Width, Height, Luma.all);
               end;
            end loop;
            Free (Luma);
            declare
               F : constant Follow.Followed := Follow.Run (Views, Track (1 .. Count), Open_Mask);
            begin
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & P.Group'Image & " channel" & C'Image & " lobe"
                                & Lobe_No'Image & ": followed" & F.Count'Image & " of its" & F.Asked'Image
                                & " points through" & Count'Image & " views from the open end to the closed one"
                                & (if F.Count > 0 then ", the matcher's noise " & Driver.Log.Image (F.Sigma, 3) & " pixels"
                                   else ": " & Ada.Strings.Unbounded.To_String (F.Why))
                                & (if F.Worst > 0 then "; most lost at the step from view" & F.Worst'Image & ":"
                                        & Ada.Strings.Unbounded.To_String (F.Lost)
                                   else ""));
               if F.Count = 0 then
                  return Shapes.Not_Followed ("its pixels are not followed from the open end to the closed one: "
                                          & Ada.Strings.Unbounded.To_String (F.Why));
               end if;
               declare
                  Followed_Moves : Driver.Robot.Hand.Lobes.Move_Array renames F.Moves.Constant_Reference.Element.all;
                  Moves : Driver.Robot.Hand.Lobes.Move_Array (1 .. F.Count + 1);
               begin
                  --  The tip first: the point the presses measure at the open end, the shape's first anchor; the
                  --  second, the tip at the closed end, or where it was last followed (Closed_Is_End).
                  Moves (1) := (From => Open_Tip, To => (if Partial then Tips (Count) else Closed_Tip));
                  Moves (2 .. F.Count + 1) := Followed_Moves;
                  declare
                     Seen : constant Shapes.Sighting_Array :=
                       Shapes.From_Moves (Moves, (From => (if Partial then Tips (Count) else Closed_Tip), To => Open_Tip),
                                          Line'Access, F.Sigma);
                  begin
                     return Shapes.Fit (Mount.Pose.Translation, Seen, 1, Seen'Last, Bordered, Closed_Is_End => not Partial,
                                        Pixels => Open_Mask);
                  end;
               end;
            end;
         end;
      end Followed_Shape;

      --  The way the finger closes in the picture: from its open tip to its closed one.
      function Closing_Way_Of (From, To : Driver.Images.Pixel) return Closing_Way is
         Reach : constant Real := Sqrt ((To.U - From.U) ** 2 + (To.V - From.V) ** 2);
      begin
         if Reach = 0.0 then
            return (others => <>);
         end if;
         return (Known => True, U => (To.U - From.U) / Reach, V => (To.V - From.V) / Reach, Reach => Reach);
      end Closing_Way_Of;

      --  What a lobe's tip looked like in a view where it stood still: the points of its edge in the
      --  region at its tip.
      function Patch_At
        (Of_View : Driver.Robot.Hand.Views.View;
         Lobe    : Driver.Images.Mask;
         Centre  : Driver.Images.Pixel;
         Tip     : Driver.Images.Pixel;
         Way     : Closing_Way) return Driver.Robot.Hand.Slide.Patch
      is
         Cap : constant Driver.Images.Mask := Driver.Robot.Hand.Lobes.Tip_Cap (Lobe, Centre, Tip);
      begin
         return Driver.Robot.Hand.Slide.Take (Lobe, Cap, Of_View.Frames, Way.U, Way.V, Way.Reach);
      end Patch_At;
   begin
      for C in 1 .. Sweeps.Channels (P.Sweep) loop
         if Sweeps.Status (P.Sweep, C) = Sweeps.Measured and then Sweeps.Closing_Known (P.Sweep, C) then
            declare
               Low  : constant Driver.Robot.Hand.Views.View := Sweeps.Low_End (P.Sweep, C);
               High : constant Driver.Robot.Hand.Views.View := Sweeps.High_End (P.Sweep, C);
               Closed_High : constant Boolean := Sweeps.Closed_End_Is_High (P.Sweep, C);
               Lobe_No : Natural := 0;
            begin
               for L of Sweeps.Lobes_Of (P.Sweep, C) loop
                  Lobe_No := Lobe_No + 1;
                  declare
                     At_Low  : constant Sight := Seen_Tip (Low, L.Tip_Known_Here, L.Tip_Here, L.Here, L.Centre_Here);
                     At_High : constant Sight := Seen_Tip (High, L.Tip_Known_There, L.Tip_There, L.There, L.Centre_There);
                     Way     : constant Closing_Way :=
                       (if L.Tip_Known_Here and then L.Tip_Known_There
                        then Closing_Way_Of (From => (if Closed_High then L.Tip_Here else L.Tip_There),
                                             To   => (if Closed_High then L.Tip_There else L.Tip_Here))
                        else (others => <>));
                     Stops   : Stop_Vectors.Vector;
                  begin
                     --  The stops of the sweep between the ends: the lobe's tip as the eye saw it at each.
                     for K in 1 .. Sweeps.Stops (P.Sweep, C) loop
                        declare
                           V    : constant Driver.Robot.Hand.Views.View := Sweeps.Stop_View (P.Sweep, C, K);
                           Tips : constant Sweeps.Stop_Tips := Sweeps.Tips_At_Stop (P.Sweep, C, K);
                        begin
                           if Lobe_No <= Tips'Last then
                              declare
                                 T : Sweeps.Stop_Tip renames Tips (Lobe_No);
                              begin
                                 Stops.Append
                                   (Stop_Sight'(Reading => Sweeps.Stop_Reading (P.Sweep, C, K),
                                                Sight   => Seen_Tip (V, T.Known, T.Tip, T.Region, T.Centre),
                                                Patch   => (if Way.Known and then T.Known
                                                            then Patch_At (V, T.Region, T.Centre, T.Tip, Way)
                                                            else Driver.Robot.Hand.Slide.Empty)));
                              end;
                           end if;
                        end;
                     end loop;
                     Made.Lobes.Append
                       (Lobe_Record'(Channel => C,
                                     Sights  => [Open         => (if Closed_High then At_Low else At_High),
                                                 Closed_Empty => (if Closed_High then At_High else At_Low)],
                                     Stops   => Stops,
                                     Patches =>
                                       [Open         => (if Way.Known
                                                         then Patch_At ((if Closed_High then Low else High),
                                                                        (if Closed_High then L.Here else L.There),
                                                                        (if Closed_High then L.Centre_Here
                                                                         else L.Centre_There),
                                                                        (if Closed_High then L.Tip_Here else L.Tip_There),
                                                                        Way)
                                                         else Driver.Robot.Hand.Slide.Empty),
                                        Closed_Empty => (if Way.Known
                                                         then Patch_At ((if Closed_High then High else Low),
                                                                        (if Closed_High then L.There else L.Here),
                                                                        (if Closed_High then L.Centre_There
                                                                         else L.Centre_Here),
                                                                        (if Closed_High then L.Tip_There else L.Tip_Here),
                                                                        Way)
                                                         else Driver.Robot.Hand.Slide.Empty)],
                                     Way     => Way,
                                     Shape   => Followed_Shape (C, Lobe_No, L, Low, High, Closed_High),
                                     Size    => <>));
                  end;
               end loop;
               --  A jaw that closes onto a fixed one has a finger no channel moves: the fixed jaw, which the moving
               --  jaw's lead meets at the closed end and not at the open one (Lobes.Lead_Gaps against the robot's
               --  still pixels). Its tip is where the moving tip meets it, going on the way the jaw closes from where
               --  the closed end shows it: the first of the robot's still pixels straight ahead; the lobe whose lead
               --  meets them nearest gives it. It is pressed as any finger is, along that pixel's line of sight, the
               --  same at either opening (Channel 0, seen). Unknown, every touch went to the moving jaw: u7 (servo
               --  seed 3 of the fake robot, 10-19) pressed its hand onto a slab on the table with the fixed jaw, the
               --  moving one 6 mm above it by the truth, and put the moving jaw's tip 10.8 cm from its eye along its
               --  line of sight where it is 5.2.
               declare
                  Still    : constant Driver.Images.Mask := Sweeps.Located_Of (P.Sweep, C).Still;
                  W        : constant Natural := Driver.Images.Width (Still);
                  H        : constant Natural := Driver.Images.Height (Still);
                  Nearest  : Real := Real'Last;
                  Fixed_Px : Driver.Images.Pixel := (U => 0.0, V => 0.0);
                  Found    : Boolean := False;
                  Part     : Driver.Images.Mask;   --  the fixed jaw: the part of them the moving jaw meets
                  Part_Centre, Part_Tip : Driver.Images.Pixel := (U => 0.0, V => 0.0);
                  Still_N  : Natural := 0;
               begin
                  for Y in 0 .. H - 1 loop
                     for X in 0 .. W - 1 loop
                        if Driver.Images.Contains (Still, X, Y) then
                           Still_N := Still_N + 1;
                        end if;
                     end loop;
                  end loop;
                  if Still_N > 0 then
                     for L of Sweeps.Lobes_Of (P.Sweep, C) loop
                        declare
                           At_Low, At_High, Never : Real;
                           Tip   : constant Driver.Images.Pixel := (if Closed_High then L.Tip_There else L.Tip_Here);
                           Known : constant Boolean := (if Closed_High then L.Tip_Known_There else L.Tip_Known_Here);
                           --  The way the jaw closes: the lobe's way from the low view to the high one, turned round
                           --  when it is shut at the low reading (Lead_Gaps' convention).
                           Du    : constant Real := (if L.Way_Known then L.Way_U else L.Centre_There.U - L.Centre_Here.U);
                           Dv    : constant Real := (if L.Way_Known then L.Way_V else L.Centre_There.V - L.Centre_Here.V);
                           Size  : constant Real := Sqrt (Du * Du + Dv * Dv);
                        begin
                           Driver.Robot.Hand.Lobes.Lead_Gaps (L, Still, At_Low, At_High, Never);
                           declare
                              Shut_Gap : constant Real := (if Closed_High then At_High else At_Low);
                              Open_Gap : constant Real := (if Closed_High then At_Low else At_High);
                              Su       : constant Real := (if Size > 0.0 then (if Closed_High then Du else -Du) / Size else 0.0);
                              Sv       : constant Real := (if Size > 0.0 then (if Closed_High then Dv else -Dv) / Size else 0.0);
                           begin
                              if Known and then Size > 0.0 and then Shut_Gap < Never and then Shut_Gap < Open_Gap
                                and then Shut_Gap < Nearest
                              then
                                 for K in 1 .. Natural (Real'Floor (Never)) loop
                                    declare
                                       Xk : constant Real := Real'Floor (Tip.U + Real (K) * Su);
                                       Yk : constant Real := Real'Floor (Tip.V + Real (K) * Sv);
                                    begin
                                       exit when Xk < 0.0 or else Yk < 0.0 or else Xk >= Real (W) or else Yk >= Real (H);
                                       if Driver.Images.Contains (Still, Natural (Xk), Natural (Yk)) then
                                          Nearest := Shut_Gap;
                                          Fixed_Px := (U => Xk + 0.5, V => Yk + 0.5);
                                          Found := True;
                                          exit;
                                       end if;
                                    end;
                                 end loop;
                              end if;
                           end;
                        end;
                     end loop;
                  end if;
                  --  The fixed jaw is the part of the still pixels the moving jaw closes onto, and its tip is that part's
                  --  farthest pixel from the picture's border, as a moving lobe's is (Lobes.Part_Of). Taken as the
                  --  pixel the moving jaw met on its inner edge, with every still pixel of the robot for its region, its
                  --  tip region was 4.8 wide where the moving jaw's was 0.8, and every tilted aim of it was refused for
                  --  the tracks standing within 19 of where it would land (u14, servo seed 3 of the fake robot, both
                  --  arms, 10-19).
                  if Found then
                     Driver.Robot.Hand.Lobes.Part_Of (Still, Fixed_Px, Part, Part_Centre, Part_Tip, Found);
                  end if;
                  if Found then
                     declare
                        Closed : constant Driver.Robot.Hand.Views.View := (if Closed_High then High else Low);
                        Seen   : constant Sight := Seen_Tip (Closed, True, Part_Tip, Part, Part_Centre);
                     begin
                        Made.Lobes.Append
                          (Lobe_Record'(Channel => 0,
                                        Sights  => [others => Seen],
                                        Stops   => <>,
                                        Patches => [others => Driver.Robot.Hand.Slide.Empty],
                                        Way     => <>,
                                        Shape   => Driver.Robot.Hand.Shape.Unfitted
                                                     ("a fixed jaw: no channel moves it; its tip is where the moving jaw "
                                                      & "closes onto it"),
                                        Size    => <>));
                        Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & P.Group'Image & " channel" & C'Image
                                         & " closes a jaw onto a fixed one, its lead " & Driver.Log.Image (Nearest, 1)
                                         & " px from the robot's still pixels at the closed end: a finger no channel"
                                         & " moves, its tip at " & Image (Part_Tip) & ", the moving jaw meeting it at "
                                         & Image (Fixed_Px));
                     end;
                  end if;
               end;
               --  The group's readings at each opening: this channel at its end,
               --  the others as they were held while it was swept.
               for Which in Opening loop
                  declare
                     At_End : constant Real_Array :=
                       (if (Which = Open) = Closed_High then Low.Closer.Element else High.Closer.Element);
                  begin
                     if Made.Readings (Which).Is_Empty then
                        Made.Readings (Which) := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (At_End);
                     else
                        declare
                           Merged : Real_Array := Made.Readings (Which).Element;
                        begin
                           Merged (Merged'First + C - 1) := At_End (At_End'First + C - 1);
                           Made.Readings (Which) := Driver.Robot.Hand.Views.Reading_Holders.To_Holder (Merged);
                        end;
                     end if;
                  end;
               end loop;
            end;
         end if;
      end loop;
      if not Made.Lobes.Is_Empty then
         Keep (D, Made);
      else
         --  Ends measured again and no lobes this time: the views of them that came since did not tell the robot
         --  from its surroundings, which says nothing against what the views before told; a hand made of them
         --  stands. s46 (servo seed 6 of the fake robot, 10-17): one lobe at beat 4844, its tip from (267.5, 74.5)
         --  to (183.5, 58.5), 138 of 138 points followed; at 4851, the eye raised once, the views did not fall in
         --  two groups, the hand was dropped, and nothing was pressed. Kept only where the ends were measured
         --  again at the very readings of the hand (a2bf1aa3), it was dropped in s53 (seed 6, 10-18): two lobes at
         --  beat 458, the ends seen again at other readings at 633 gave none, and the brain had no hand at all.
         for Id in D.Found.First_Index .. D.Found.Last_Index loop
            if D.Found (Id).Group = P.Group then
               Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & P.Group'Image & ": its ends, measured "
                                & "again, gave no lobes; the hand from before stands");
            end if;
         end loop;
      end if;
   end Rebuild;

   procedure Adopt
     (H         : in out Hands;
      Group     : Group_Id;
      Arm       : Arm_Id;
      Eye       : Eye_Id;
      Open_At   : Real_Array;
      Closed_At : Real_Array;
      Lobes     : Sight_Rows)
   is
      Made : Hand_Record := (Group => Group, Arm => Arm, Eye => Eye, others => <>);
   begin
      if H.Data = null then
         H.Data := new Hand_Data;
      end if;
      Made.Readings := [Open         => Driver.Robot.Hand.Views.Reading_Holders.To_Holder (Open_At),
                        Closed_Empty => Driver.Robot.Hand.Views.Reading_Holders.To_Holder (Closed_At)];
      for Row of Lobes loop
         Made.Lobes.Append
           (Lobe_Record'(Channel => 1,
                         Sights  => Row,
                         Stops   => <>,
                         Patches => [others => Driver.Robot.Hand.Slide.Empty],
                         Way     => <>,
                         Shape   => Driver.Robot.Hand.Shape.Unfitted ("the hand was given, not swept"),
                         Size    => <>));
      end loop;
      if not Made.Lobes.Is_Empty then
         Keep (H.Data.all, Made);
      end if;
   end Adopt;

   procedure Report (P : in out Pair; D : in out Hand_Data; M : Model; O : Observation);
   --  Says what became of every channel whose ends are new since it last
   --  said, and makes the hand of the measured ones again: a hand found from
   --  older ends does not stand when the ends are new and give no lobes. Made
   --  again too when the eye's place in the tool frame has become known
   --  (Pair.Placed_Made).

   procedure Report (P : in out Pair; D : in out Hand_Data; M : Model; O : Observation) is
      Said   : Boolean := False;
      Placed : constant Boolean := Eye_In_Tool (M, P.Eye, O).Position_Covariance (1, 1) < Real'Last;
   begin
      for C in 1 .. Sweeps.Channels (P.Sweep) loop
         if Sweeps.Unannounced (P.Sweep, C) then
            Sweeps.Announce (P.Sweep, C);
            Driver.Log.Line (Driver.Log.Robot, "hand: closer group" & P.Group'Image & " channel" & C'Image & " in eye"
                             & P.Eye'Image & ": " & Sweeps.Account (P.Sweep, C));
            if Sweeps.Status (P.Sweep, C) = Sweeps.Measured then
               declare
                  Number : Natural := 0;
               begin
                  for L of Sweeps.Lobes_Of (P.Sweep, C) loop
                     Number := Number + 1;
                     Driver.Log.Line
                       (Driver.Log.Robot, "hand: closer group" & P.Group'Image & " channel" & C'Image & " in eye"
                        & P.Eye'Image & ": lobe" & Number'Image & ":" & L.Count_Here'Image & " pixels at the low reading, "
                        & (if L.Tip_Known_Here then "tip " & Image (L.Tip_Here) else "no tip")
                        & (if L.Bordered_Here then " from the border" else "") & ";" & L.Count_There'Image
                        & " at the high reading, " & (if L.Tip_Known_There then "tip " & Image (L.Tip_There) else "no tip")
                        & (if L.Bordered_There then " from the border" else ""));
                  end loop;
               end;
            end if;
            Said := True;
         end if;
      end loop;
      declare
         Stops_Now : Natural := 0;
         Measured  : Boolean := False;   --  some channel's lobes to make the hand of
      begin
         for C in 1 .. Sweeps.Channels (P.Sweep) loop
            if Sweeps.Status (P.Sweep, C) = Sweeps.Measured then
               Stops_Now := Stops_Now + Sweeps.Stops (P.Sweep, C);
               Measured := True;
            end if;
         end loop;
         if Said or else Stops_Now /= P.Stops_Made or else (Measured and then Placed /= P.Placed_Made) then
            P.Stops_Made := Stops_Now;
            P.Placed_Made := Placed;
            Rebuild (D, P, M);
         end if;
      end;
   end Report;

   function Opening_Of (R : Hand_Record; M : Model; Readings : Real_Array; Which : out Opening) return Boolean;
   --  The opening the closer was at, when its readings equal those of one
   --  opening within their noise and not those of the other.

   function Opening_Of (R : Hand_Record; M : Model; Readings : Real_Array; Which : out Opening) return Boolean is
      Sigma : constant Real_Array := (if R.Group = 0 then [1 .. 0 => 0.0] else Closer_Sigma (M, R.Group));
      function At_It (O : Opening) return Boolean is
         Measured : constant Real_Array := R.Readings (O).Element;
      begin
         if Measured'Length /= Readings'Length then
            return False;
         end if;
         --  Each reading and the one measured at the opening carry the channel's sigma as the body's one test of
         --  motion sees it (Closer_Sigma), so their difference carries it twice over.
         return (for all C in 1 .. Readings'Length =>
                   not Significant (Readings (Readings'First + C - 1) - Measured (Measured'First + C - 1),
                                    Sqrt (2.0) * Sigma (C)));
      end At_It;
   begin
      Which := Open;
      --  A tool no closer moves has one setting: it is always at it.
      if R.Group = 0 then
         return True;
      end if;
      if R.Readings (Open).Is_Empty or else R.Readings (Closed_Empty).Is_Empty then
         return False;
      end if;
      if At_It (Open) and then not At_It (Closed_Empty) then
         Which := Open;
         return True;
      elsif At_It (Closed_Empty) and then not At_It (Open) then
         Which := Closed_Empty;
         return True;
      end if;
      return False;
   end Opening_Of;

   function Tips_Said (R : Hand_Record; Which : Opening) return String;
   --  What the presses make of each lobe's tip at an opening: how far along
   --  its line of sight in the arm's unit, and whether a second press from
   --  another pose has landed on it (confirmed) or not (provisional).

   function Tips_Said (R : Hand_Record; Which : Opening) return String is
      Text : Unbounded_String;
   begin
      for L in 1 .. Natural (R.Lobes.Length) loop
         declare
            Reach  : constant Estimate := Driver.Robot.Hand.Tips.Distance (R.Book, L, Which);
            Across : constant Real_Array := Driver.Robot.Hand.Tips.Across (R.Book, L, Which);
         begin
            Append (Text, (if L > 1 then ", " else "") & "lobe" & L'Image & " "
                    & (if not Known (Reach) then "none"
                       else (if Driver.Robot.Hand.Tips.Confirmed (R.Book, L, Which) then "confirmed " else "provisional ")
                            & Driver.Log.Image (Reach.Value, 4) & " +- " & Driver.Log.Image (Reach.Sigma, 4)
                            & " along its sight, on" & Driver.Robot.Hand.Tips.Agreeing (R.Book, L, Which)'Image
                            & " presses, its tip region" & Driver.Log.Image (Reach.Value * R.Lobes (L).Sights (Which).Spread, 3)
                            & " wide across it, left" & Driver.Log.Image (Across (1), 3) & " and" & Driver.Log.Image (Across (2), 3)
                            & " along the axes the tilts of the presses told most and least ("
                            & (if Driver.Robot.Hand.Tips.Tested (R.Book, L, Which) then "tested across the sight"
                               else "not tested across the sight") & ")"));
         end;
      end loop;
      return To_String (Text);
   end Tips_Said;

   function Measured_Slides
     (R : Hand_Record; Id : Hand_Id; M : Model; O : Observation; Which : Opening) return Driver.Robot.Hand.Tips.Slid_Row;
   --  How far each lobe's finger stands from where the closer's reading puts
   --  it, in the picture of the beat of a press made at that opening: the
   --  pixels of its tip as the sweep saw them, found again under the press
   --  (Driver.Robot.Hand.Slide). Said in the log for every lobe, measured or not.

   function Measured_Slides
     (R : Hand_Record; Id : Hand_Id; M : Model; O : Observation; Which : Opening) return Driver.Robot.Hand.Tips.Slid_Row
   is
      Result : Driver.Robot.Hand.Tips.Slid_Row (1 .. Natural (R.Lobes.Length)) := [others => <>];
      Text   : Unbounded_String;
   begin
      if O.Images.Is_Empty or else R.Eye > O.Images.Last_Index or else not Driver.Observations.Has_Image (O, R.Eye) then
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": no picture at the press: no slide measured");
         return Result;
      end if;
      declare
         Under : constant Driver.Robot.Hand.Slide.Picture := Driver.Robot.Hand.Slide.See (O.Images (R.Eye));
      begin
         for L in Result'Range loop
            declare
               Lobe  : constant Lobe_Record := R.Lobes (L);
               Shift : Driver.Robot.Hand.Slide.Shift;
            begin
               Append (Text, (if L > 1 then "; " else "") & "lobe" & L'Image & " ");
               if not Lobe.Way.Known or else Driver.Robot.Hand.Slide.Points (Lobe.Patches (Which)) = 0 then
                  Append (Text, "has no edge to look for");
               else
                  Driver.Robot.Hand.Slide.Measure (Lobe.Patches (Which), Under, Shift);
                  if Shift.Known then
                     declare
                        --  The tip's line of sight under the press: the sweep's tip pixel moved by the slide along the way
                        --  the finger closes, in the tool frame (the eye is the tool frame of the arm that carries it); the
                        --  slide's sigma at the pitch of a pixel there is how far the finger may still be off it (item 8).
                        Px    : constant Driver.Images.Pixel := Lobe.Sights (Which).Pixel;
                        Slid  : constant Ray_Estimate :=
                          Driver.Robot.Hand.Frames.Into
                            (Eye_In_Tool (M, R.Eye, O),
                             Eye_Ray (M, R.Eye, (U => Px.U + Shift.By * Lobe.Way.U, V => Px.V + Shift.By * Lobe.Way.V)));
                        Lined : constant Boolean :=
                          Slid.Direction.Sigma < Real'Last and then abs Slid.Direction.Unit_Vector > 0.0;
                     begin
                        Result (L) := (Known          => True,
                                       Pixels         => Shift.By,
                                       Pixels_Sigma   => Shift.Sigma,
                                       Fraction       => Shift.By / Lobe.Way.Reach,
                                       Fraction_Sigma => Shift.Sigma / Lobe.Way.Reach,
                                       Lined          => Lined,
                                       Line           => (if Lined then Slid.Direction.Unit_Vector else [0.0, 0.0, 0.0]),
                                       Line_Sigma     => (if Lined then Lobe.Sights (Which).Pitch * Shift.Sigma else Real'Last));
                     end;
                     Append (Text, "slid" & Driver.Log.Image (Shift.By, 2) & " +- " & Driver.Log.Image (Shift.Sigma, 2)
                             & " pixels, " & Driver.Log.Image (100.0 * Shift.By / Lobe.Way.Reach, 2) & " % of the"
                             & Driver.Log.Image (Lobe.Way.Reach, 1) & " it closes");
                  else
                     Append (Text, "not found at" & Driver.Log.Image (Shift.By, 2) & " pixels");
                  end if;
                  Append (Text, " (the edge steps by" & Driver.Log.Image (Shift.Peak, 1) & " levels there,"
                          & Driver.Log.Image (Shift.Still, 1) & " where it stood, the others by" & Driver.Log.Image (Shift.Typical, 1)
                          & " about their middle)");
               end if;
            end;
         end loop;
      end;
      Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": under the press at the " & Opening'Image (Which)
                       & " opening, at beat" & O.Beat'Image & ", the fingers stand from where the reading puts them: "
                       & To_String (Text));
      return Result;
   end Measured_Slides;

   procedure Judge_Push (R : in out Hand_Record; Id : Hand_Id; M : Model; O : Observation);
   --  Judges the arm's latest push, once it has ended at rest, by where its
   --  readings put the points of the hand (Driver.Robot.Hand.Lowering): the
   --  tool's origin and the tips measured at the opening the closer stands at.
   --  A push the arm followed and the hand did not go down with is a stall:
   --  the hand lies on what it met, as a blocked push says too (A17's third
   --  press, 4,600 beats on the table). Read from the stream alone, so that a
   --  replay gives the verdicts of the run it replays.

   procedure Judge_Push (R : in out Hand_Record; Id : Hand_Id; M : Model; O : Observation) is
      Arm_Is : constant Group_Id := Arm_Group (M, R.Arm);
   begin
      if Driver.Robot.Steps.Episodes (M, Arm_Is) = 0 then
         return;
      end if;
      declare
         E : constant Episode := Driver.Robot.Steps.Latest (M, Arm_Is);
      begin
         if not (E.Ended and then E.Settled and then E.Rested) or else E.Start = R.Judged then
            return;
         end if;
         R.Judged := E.Start;
         if E.Start < 1 or else not Driver.Robot.Channels.Has_Reading (M, Arm_Is, E.Start - 1)
           or else not Driver.Robot.Channels.Has_Reading (M, Arm_Is, E.End_At)
           or else not Driver.Robot.Channels.Has_Target (M, Arm_Is, E.Start)
         then
            return;
         end if;
         declare
            Size   : constant Natural := Group_Size (M, Arm_Is);
            Up     : constant Direction_Estimate := Up_In_Arm (M, R.Arm);
            Before : constant Real_Array :=
              [for C in 1 .. Size => Driver.Robot.Channels.Reading (M, Arm_Is, E.Start - 1, C)];
            Asked  : constant Real_Array := [for C in 1 .. Size => Driver.Robot.Channels.Target (M, Arm_Is, E.Start, C)];
            After  : constant Real_Array := [for C in 1 .. Size => Driver.Robot.Channels.Reading (M, Arm_Is, E.End_At, C)];

            function Seen_At (Arm : Real_Array) return Observation is
               Then_Read : Observation;
            begin
               Then_Read.Readings := O.Readings;
               Then_Read.Readings.Replace_Element (Arm_Is, Arm);
               return Then_Read;
            end Seen_At;

            function Norm (V : Real_Array) return Real is
               Sum : Real := 0.0;
            begin
               for X of V loop
                  Sum := Sum + X * X;
               end loop;
               return Sqrt (Sum);
            end Norm;

            --  What the readings did: how long the push asked, and how far short of its target they stopped, the
            --  whole vector and not only the part along the ask.
            Ask_Vec   : constant Real_Array := [for C in 1 .. Size => Asked (C) - Before (C)];
            Short_Vec : constant Real_Array := [for C in 1 .. Size => Asked (C) - After (C)];
            Joints    : constant Driver.Robot.Hand.Lowering.Joint_Push :=
              (Asked  => Norm (Ask_Vec) > 0.0 and then Driver.Robot.Channels.Visible (M, Arm_Is, Ask_Vec),
               Length => Norm (Ask_Vec),
               Short  => Norm (Short_Vec),
               Seen   => Driver.Robot.Channels.Visible (M, Arm_Is, Short_Vec));

            From   : constant Pose_Estimate := Tool_In_Arm (M, R.Arm, Seen_At (Before));
            Target : constant Pose_Estimate := Tool_In_Arm (M, R.Arm, Seen_At (Asked));
            To     : constant Pose_Estimate := Tool_In_Arm (M, R.Arm, Seen_At (After));
         begin
            if Up.Sigma = Real'Last or else From.Position_Covariance (1, 1) = Real'Last
              or else Target.Position_Covariance (1, 1) = Real'Last or else To.Position_Covariance (1, 1) = Real'Last
            then
               return;
            end if;
            --  A push whose target stands the tool where it stood, as far as its place tells from its noise
            --  (Pressing.Least_Push), asked the hand nothing, and a share of nothing is no number: A79's aim of a
            --  press, whose push began at a target 3e-8 from where the tool stood while the arm went on 2.6 of
            --  its units the way a joint's end let it, was judged a stall of 95,629,588 per cent, and the rest
            --  after it was kept as a press of lobe 2 at closed, a tip 45 +- 89 units along its sight (760 mm).
            if abs (Target.Pose.Translation - From.Pose.Translation)
                 < Driver.Robot.Hand.Pressing.Least_Push (M, R.Arm, Seen_At (Before))
            then
               return;
            end if;
            declare
               Closer : constant Natural := (if R.Group = 0 then 0 else Group_Size (M, R.Group));
               Lobes  : constant Natural := Natural (R.Lobes.Length);
               Where  : Driver.Robot.Hand.Lowering.Points (1 .. 1 + Lobes) := [others => Zero3];
               Whose  : array (1 .. 1 + Lobes) of Natural := [others => 0];   --  the lobe a point is the tip of
               Count  : Natural := 1;
               Which  : Opening;
               Said   : Driver.Robot.Hand.Lowering.Judgment;
            begin
               if (R.Group = 0 or else Driver.Robot.Channels.Has_Reading (M, R.Group, E.End_At))
                 and then Opening_Of (R, M, [for C in 1 .. Closer => Driver.Robot.Channels.Reading (M, R.Group, E.End_At, C)],
                                      Which)
               then
                  for L in 1 .. Lobes loop
                     declare
                        Tip : constant Point_Estimate := Driver.Robot.Hand.Tips.Tip (R.Book, L, Which);
                     begin
                        if Known (Tip) then
                           Count := Count + 1;
                           Where (Count) := Tip.Mean;
                           Whose (Count) := L;
                        end if;
                     end;
                  end loop;
               end if;
               Driver.Robot.Hand.Lowering.Judge
                 (R.Lowering, From, Target, To, -Up.Unit_Vector, Where (1 .. Count), Joints, Said);
               if Said.Result = Driver.Robot.Hand.Lowering.Stalled then
                  R.Stalled := E.Start;
                  R.Stalls := R.Stalls + 1;
                  declare
                     Pushes  : constant String := Driver.Robot.Hand.Lowering.Pushes (R.Lowering)'Image;
                     Said_By : Unbounded_String;
                  begin
                     if Said.Joint_Stalled then
                        Append (Said_By, " its readings stopped short of the push's target by "
                                & Driver.Log.Image (100.0 * Said.Joint_Share, 1) & " % of its length, where the" & Pushes
                                & " pushes of this descent before it stopped short by at most "
                                & Driver.Log.Image (100.0 * Said.Joint_Free, 3) & " %;");
                     end if;
                     if Said.Point_Stalled then
                        Append (Said_By, " asked to take " & (if Said.Point = 1 then "the tool's origin" else "the tip of lobe"
                                                              & Whose (Said.Point)'Image)
                                & " down" & Driver.Log.Image (Said.Asked, 4) & ", it went down" & Driver.Log.Image (Said.Went, 4)
                                & ", short by " & Driver.Log.Image (100.0 * Said.Share, 1) & " % of the ask, where the pushes"
                                & " before it fell short by at most " & Driver.Log.Image (100.0 * Said.Free, 3) & " %;");
                     end if;
                     Driver.Log.Line
                       (Driver.Log.Robot,
                        "hand" & Id'Image & ": the arm followed the push that began at beat" & E.Start'Image
                        & " and the hand did not go down with it:" & To_String (Said_By) & " it has stopped lowering the hand");
                  end;
               end if;
            end;
         end;
      end;
   end Judge_Push;

   procedure Watch
     (R          : in out Hand_Record;
      Id         : Hand_Id;
      M          : Model;
      O          : Observation;
      Is_Blocked : Boolean;
      Is_Still   : Boolean;
      Is_Pushing : Boolean);
   --  Follows the hand's arm for presses and keeps every press made at one
   --  of the hand's openings. Presses are found in the arm's own frame, the
   --  one its fit gives the tool pose (Tool_In_Arm) and the table its eye saw
   --  (Table_In_Arm) in: they need neither where the arm stands in the world
   --  nor the arm's unit, and the surface the presses fit is that table
   --  corrected by them. The arm is fitted again as it moves, and its frame
   --  and unit with it: when its table is not the one the presses were fitted
   --  with, they take their poses again from the arm's readings they kept.

   procedure Watch
     (R          : in out Hand_Record;
      Id         : Hand_Id;
      M          : Model;
      O          : Observation;
      Is_Blocked : Boolean;
      Is_Still   : Boolean;
      Is_Pushing : Boolean)
   is
      Found : Boolean;
      Moved : Boolean;
      Press : Driver.Robot.Hand.Presses.Event;
      Which : Opening;
      Pushed_Off : Boolean;   --  what the press met pushed the closer off the opening it stood at

      --  The arm's latest push is one judged stalled.
      function Stalled_Now return Boolean is
        (R.Stalled /= 0 and then Driver.Robot.Steps.Episodes (M, Arm_Group (M, R.Arm)) > 0
         and then Driver.Robot.Steps.Latest (M, Arm_Group (M, R.Arm)).Start = R.Stalled);

      --  The tool's displacement between two of the arm's readings, as one fit puts both (Tool_Step_In_Arm).
      function Step_Of (From, To : Real_Array) return Point_Estimate is (Tool_Step_In_Arm (M, R.Arm, From, To));

      function Pose_Of (Arm : Real_Array) return Pose_Estimate is
         Then_Read : Observation;
      begin
         Then_Read.Readings := O.Readings;
         Then_Read.Readings.Replace_Element (Arm_Group (M, R.Arm), Arm);
         return Tool_In_Arm (M, R.Arm, Then_Read);
      end Pose_Of;
   begin
      Driver.Robot.Hand.Tips.Set_Frame (R.Book, Table_In_Arm (M, R.Arm), Pose_Of'Access, Moved);
      if Moved and then Driver.Robot.Hand.Tips.Pressed (R.Book) > 0 then
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": its arm was fitted again; the"
                          & Driver.Robot.Hand.Tips.Pressed (R.Book)'Image & " presses kept take their poses from the new fit");
         Size_Up (R, Id);
      end if;
      Judge_Push (R, Id, M, O);
      if R.Group > 0 and then not Driver.Observations.Has_Reading (O, R.Group) then
         return;
      end if;
      --  A push that stalled is a block as far as the press is concerned: the verdict stands until the next push begins.
      --  A new command's readings, as a tool pose, tell a let-go from a retreat (Presses).
      declare
         G          : constant Group_Id := Arm_Group (M, R.Arm);
         Retargeted : constant Boolean := Driver.Robot.Channels.Target_Changed (M, G, Natural (O.Beat));
         Asked      : Pose_Estimate;
         --  The command holds the arm where it reads: no channel's target is off its reading by a step the readings
         --  tell (Channels.Visible).
         function Holding return Boolean is
         begin
            if not Driver.Robot.Channels.Has_Target (M, G, Natural (O.Beat))
              or else not Driver.Robot.Channels.Has_Reading (M, G, Natural (O.Beat))
            then
               return False;
            end if;
            return not Driver.Robot.Channels.Visible
              (M, G, [for C in 1 .. Group_Size (M, G) =>
                        Driver.Robot.Channels.Target (M, G, Natural (O.Beat), C)
                        - Driver.Robot.Channels.Reading (M, G, Natural (O.Beat), C)]);
         end Holding;
         Held       : constant Boolean := Holding;
      begin
         if Retargeted and then Driver.Robot.Channels.Has_Target (M, G, Natural (O.Beat)) then
            declare
               Wanted : Real_Array (1 .. Group_Size (M, G));
            begin
               for C in Wanted'Range loop
                  Wanted (C) := Driver.Robot.Channels.Target (M, G, Natural (O.Beat), C);
               end loop;
               Asked := Pose_Of (Wanted);
            end;
         end if;
         --  A block at a joint's end the body has shown is the body's own, no contact: no press is made of it (item 7).
         Driver.Robot.Hand.Presses.Observe (R.Watch, O.Beat,
                                            (Is_Blocked and then not Stopped_By_An_End (M, R.Arm, O)) or else Stalled_Now,
                                            Is_Pushing, Is_Still,
                                            Tool_In_Arm (M, R.Arm, O),
                                            O.Readings.Element (G),
                                            (if R.Group > 0 then O.Readings.Element (R.Group) else [1 .. 0 => 0.0]),
                                            Found, Press, Retargeted => Retargeted, Asked => Asked,
                                            Step_Of => Step_Of'Unrestricted_Access, Held => Held);
      end;
      if not Found then
         return;
      end if;
      R.Rests := R.Rests + 1;
      --  The press is the opening's the closer stood at as it began, also where what it met pushed the closer off it:
      --  the finger was loaded, as one that gives without its reading changing is, and the eye tells where it stood
      --  under the press (Measured_Slides), which the fit of the presses takes, as the finger stood loaded and as it
      --  stands free at the opening's reading (Tips). A jaw held open whose tip met the table was pressed 2 % of its
      --  travel shut (Presses.Event's Before), and every press of it at open was refused (f16u, f16y on 10-15/16).
      Pushed_Off := not Opening_Of (R, M, Press.Closer.Element, Which);
      if not Pushed_Off or else (not Press.Before.Is_Empty and then Opening_Of (R, M, Press.Before.Element, Which)) then
         Driver.Robot.Hand.Tips.Add (R.Book, Press, Which, Measured_Slides (R, Id, M, O, Which));
         Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image & ": a press at the " & Opening'Image (Which)
                          & " opening," & (if Pushed_Off then " what it met pushing the closer off it," else "")
                          & " at beat" & Press.Beat'Image & "," & Driver.Robot.Hand.Tips.Pressed (R.Book)'Image
                          & " kept; "
                          & (if Driver.Robot.Hand.Tips.Latest_Agrees (R.Book)
                             then "a tip rests on it"
                             else "no tip rests on it (it stopped short of the table, or no tip is fixed)")
                          & "; the tips at this opening: " & Tips_Said (R, Which)
                          & "; the fit of the presses: " & Driver.Robot.Hand.Tips.Fit_Account (R.Book));
         Size_Up (R, Id);
      else
         --  With the readings it was judged by: where the closer stood at the press and before it, the openings as
         --  measured, and the noise the test allows (a27 on 10-16: two presses at open dropped so, which a replay
         --  cannot show, the presses being the boot's and not the recording's).
         declare
            Sigma : constant Real_Array := Closer_Sigma (M, R.Group);
            function Image (X : Real_Array) return String is
              (if X'Length = 0 then "" else " " & Driver.Log.Image (X (X'First), 6) & Image (X (X'First + 1 .. X'Last)));
         begin
            Driver.Log.Line (Driver.Log.Robot, "hand" & Id'Image
                             & ": a press with the closer at neither measured opening is not used: the closer read"
                             & Image (Press.Closer.Element) & " at the press and"
                             & (if Press.Before.Is_Empty then " nothing" else Image (Press.Before.Element))
                             & " before it; open" & Image (R.Readings (Open).Element) & ", closed"
                             & Image (R.Readings (Closed_Empty).Element) & ", the noise" & Image (Sigma));
         end;
      end if;
   end Watch;

   procedure Press_Beat
     (H          : in out Hands;
      Id         : Hand_Id;
      M          : Model;
      O          : Observation;
      Is_Blocked : Boolean;
      Is_Still   : Boolean;
      Is_Pushing : Boolean := False)
   is
      R : Hand_Record := H.Data.Found (Id);
   begin
      Watch (R, Id, M, O, Is_Blocked, Is_Still, Is_Pushing);
      H.Data.Found.Replace_Element (Id, R);
   end Press_Beat;

   function Arm_At_Rest (M : Model; A : Arm_Id; O : Observation) return Boolean is
      G : Group_Id;
   begin
      if Natural (A) > Arm_Count (M) or else Natural (O.Beat) = 0 then
         return False;
      end if;
      G := Arm_Group (M, A);
      if not Driver.Robot.Channels.Has_Reading (M, G, Natural (O.Beat))
        or else not Driver.Robot.Channels.Has_Reading (M, G, Natural (O.Beat) - 1)
      then
         return False;
      end if;
      declare
         Before : Observation;
         Now_At, Then_At : Pose_Estimate;
         Least : constant Real := Driver.Robot.Hand.Pressing.Least_Push (M, A, O);
      begin
         Before.Readings := O.Readings;
         Before.Readings.Replace_Element
           (G, [for C in 1 .. Group_Size (M, G) => Driver.Robot.Channels.Reading (M, G, Natural (O.Beat) - 1, C)]);
         Now_At := Tool_In_Arm (M, A, O);
         Then_At := Tool_In_Arm (M, A, Before);
         return Least < Real'Last
           and then abs (Now_At.Pose.Translation - Then_At.Pose.Translation) < Least
           and then Driver.Numerics.Angle (Transpose (Then_At.Pose.Rotation) * Now_At.Pose.Rotation)
                      < Driver.Conventions.Z * Driver.Robot.Kinematics.Angle_Sigma (M, A);
      end;
   end Arm_At_Rest;
   --  The tool stood still at the beat as far as its pose is known: it moved since the beat before by less than
   --  the least move that tells from the noise of its place (Pressing.Least_Push) and turned by less than Z sigmas
   --  of its turn. The body's eyes are not asked: the pose of a press is a function of the readings alone, and the
   --  pictures settle after the arm does. The arm's readings were asked once, by the body's one test of motion, and
   --  that test sees what the eyes can tell over every cell of them: a hand resting on the table crept by a
   --  millionth of a radian a beat, more than eye 3 tells, and A78's hand 2 waited 2,900 beats after its let-go
   --  for a rest its pose had reached at the first of them. Unknown where the tool's place is not measured.

   function Asks_A_Translation (M : Model; A : Arm_Id; From, Target : Real_Array) return Boolean is
      Arm_Is : constant Group_Id := Arm_Group (M, A);

      function Pose_At (Arm : Real_Array) return Pose_Estimate is
         O : Observation;
      begin
         for G in Group_Id'First .. Arm_Is loop
            O.Readings.Append ((if G = Arm_Is then Arm else Real_Array'(1 .. 0 => 0.0)));
         end loop;
         return Tool_In_Arm (M, A, O);
      end Pose_At;

      Began : constant Pose_Estimate := Pose_At (From);
      Goal  : constant Pose_Estimate := Pose_At (Target);
   begin
      --  Nothing is told of a push whose poses are not known: it is left to the watcher, as every push was.
      return Began.Position_Covariance (1, 1) = Real'Last or else Goal.Position_Covariance (1, 1) = Real'Last
        or else Driver.Numerics.Angle (Transpose (Began.Pose.Rotation) * Goal.Pose.Rotation)
                <= Driver.Conventions.Z * Driver.Robot.Kinematics.Angle_Sigma (M, A);
   end Asks_A_Translation;

   function Latest_Push_Lowers (M : Model; A : Arm_Id) return Boolean is
      Arm_Is : constant Group_Id := Arm_Group (M, A);
   begin
      if Driver.Robot.Steps.Episodes (M, Arm_Is) = 0 then
         return True;
      end if;
      declare
         E : constant Episode := Driver.Robot.Steps.Latest (M, Arm_Is);
      begin
         if E.Start < 1 or else not Driver.Robot.Channels.Has_Reading (M, Arm_Is, E.Start - 1)
           or else not Driver.Robot.Channels.Has_Target (M, Arm_Is, E.Start)
         then
            return True;
         end if;
         declare
            Size : constant Natural := Group_Size (M, Arm_Is);
         begin
            return Asks_A_Translation
              (M, A,
               From   => [for C in 1 .. Size => Driver.Robot.Channels.Reading (M, Arm_Is, E.Start - 1, C)],
               Target => [for C in 1 .. Size => Driver.Robot.Channels.Target (M, Arm_Is, E.Start, C)]);
         end;
      end;
   end Latest_Push_Lowers;
   --  The arm's latest push, under way or ended, asked the tool a translation. A block is a press only when the push
   --  that was blocked lowered the hand: A27's first press of hand 2 was an aim the arm itself stopped (its third
   --  joint at +0.07, asked -0.05, the eye 12.39 above the table), the stream's watcher took the rest after it for a
   --  press, and a tip 12.39 from the eye was fitted to it, then confirmed by two presses stopped in the air as well.

   function Arm_Pushed (M : Model; A : Arm_Id) return Boolean is
     (Natural (A) <= Arm_Count (M)
      and then Driver.Robot.Steps.Episodes (M, Arm_Group (M, A)) > 0
      and then not Driver.Robot.Steps.Latest (M, Arm_Group (M, A)).Ended);
   --  The arm's latest push has begun and not ended.

   procedure Observe (H : in out Hands; M : Model; O : Observation; Sent : Driver.Commands.Command) is
      pragma Unreferenced (Sent);
   begin
      if H.Data = null then
         H.Data := new Hand_Data;
      end if;
      Find_Pairs (H.Data.all, M, O);
      for I in H.Data.Pairs.First_Index .. H.Data.Pairs.Last_Index loop
         declare
            P : Pair := H.Data.Pairs (I);
         begin
            Feed (P, M, O);
            Report (P, H.Data.all, M, O);
            H.Data.Pairs.Replace_Element (I, P);
         end;
      end loop;
      for Id in H.Data.Found.First_Index .. H.Data.Found.Last_Index loop
         Press_Beat (H, Id, M, O,
                     Is_Blocked => Blocked (M, H.Data.Found (Id).Arm, O)
                                   and then Latest_Push_Lowers (M, H.Data.Found (Id).Arm),
                     Is_Still   => Arm_At_Rest (M, H.Data.Found (Id).Arm, O),
                     Is_Pushing => Arm_Pushed (M, H.Data.Found (Id).Arm));
      end loop;
   end Observe;

   procedure Free_Closer
     (Arrived    : not null access function return Boolean;
      Reading    : not null access function return Real;
      Moved      : not null access function (Before, After : Real) return Boolean;
      Ask        : not null access procedure;
      Raise_Hand : not null access procedure (First : Boolean; Raised : out Boolean);
      Raises     : out Natural)
   is
      Before : Real;
      Done   : Boolean;
   begin
      Raises := 0;
      Ask.all;
      while not Arrived.all loop
         Before := Reading.all;
         Raise_Hand (Raises = 0, Done);
         exit when not Done;
         Raises := Raises + 1;
         Ask.all;
         exit when not Arrived.all and then not Moved (Before, Reading.all);
      end loop;
   end Free_Closer;

   procedure Gather_Poses
     (Wanted       : Positive;
      Wait_At_Most : Positive;
      Poses        : not null access function return Natural;
      Raise_Eye    : not null access procedure (First : Boolean; Raised : out Boolean);
      Raises       : out Natural;
      Reached      : out Boolean)
   is
      Have   : Natural;
      Raised : Boolean;
   begin
      Raises := 0;
      loop
         Have := Poses.all;
         exit when Have >= Wanted;
         Raise_Eye (Raises = 0, Raised);
         exit when not Raised;
         Raises := Raises + 1;
         --  The new pose's frame is kept once the eye's picture has rested from the move: asked a beat later each time.
         declare
            Asked : Natural := 0;
         begin
            while Asked < Wait_At_Most and then Poses.all <= Have loop
               Asked := Asked + 1;
            end loop;
         end;
      end loop;
      Reached := Poses.all >= Wanted;
   end Gather_Poses;

   procedure Sweep_Way
     (Way          : Real;
      Step         : Real;
      Seen_By      : Real;
      Wait_At_Most : Positive;
      Push         : not null access procedure (Offset : Real; Followed : out Boolean);
      Shows        : not null access function return Showing;
      Pushes       : out Natural;
      Unseen       : out Natural;
      Longest_Wait : out Natural;
      Formed       : out Boolean;
      Answered     : out Boolean)
   is
      Offset   : Real := Step;
      Before   : Real := 0.0;        --  the push before this one (0 before the first)
      Followed : Boolean;
      Shown    : Boolean := False;   --  some push of this way has shown the views something
      Seen     : Showing;
   begin
      Pushes := 0;
      Unseen := 0;
      Longest_Wait := 0;
      Formed := True;
      Answered := False;
      loop
         Push (Way * Offset, Followed);
         Pushes := Pushes + 1;
         if Pushes = 1 then
            Answered := Followed;
         end if;
         exit when not Followed;
         --  Each asking is a beat later; the view forms once the eye rests,
         --  within as long as the caller measured that to take.
         declare
            Asked : Natural := 0;
         begin
            loop
               Seen := Shows.all;
               Asked := Asked + 1;
               exit when Seen /= Not_Yet or else Asked >= Wait_At_Most;
            end loop;
            Longest_Wait := Natural'Max (Longest_Wait, Asked);
         end;
         if Seen = Not_Yet then
            Formed := False;
            exit;
         end if;
         if Seen = Something_New then
            Shown := True;
         else
            Unseen := Unseen + Boolean'Pos (not Shown);
            --  Nothing new: stuck when nothing was ever shown and a push that moves the view by a pixel in all shows
            --  nothing; past the end once something was shown and a push that moves it a pixel beyond the push before
            --  shows nothing new: below that a closer still moving shows nothing new. f16k's jaw (10-15) "showed" ten
            --  pixels at its first push of 0.0007 units, nothing new at the next two (a pixel is 0.15 units), and the
            --  way ended there, its two ends 0.003 units apart; f16w's servo jaw ended at the push of 19.6 units, 9.8
            --  beyond the one before, where a pixel is 16.6, its ends 16 units apart of its thousand; nothing was told
            --  apart between them, and no hand was made.
            exit when (if Shown then Offset - Before else Offset) >= Seen_By;
         end if;
         Before := Offset;
         Offset := 2.0 * Offset;
      end loop;
   end Sweep_Way;

   function In_Free_Air (Stopped : Heights) return Boolean is
     (Known (Stopped.Tip) and then Stopped.Tip.Value > Driver.Conventions.Z * Stopped.Tip.Sigma);

   procedure Descend
     (Above : not null access function return Heights;
      Least : Real;
      Lower : not null access procedure (By : Real; Outcome : out Lowered);
      Back  : not null access procedure (To_Start : Boolean; Reached : out Boolean);
      Steps : out Descent_Steps)
   is
      Fast    : Real := Least;   --  the next step of the fast part, doubling
      Past    : Real := 0.0;     --  the last step past the band, doubling
      Outcome : Lowered;
      Allowed : Natural := Natural'Last;   --  the steps the schedule takes to cover the eye's room: no more are made
      Cut     : Boolean := False;   --  the last step was cut to the room left
      Seen    : Heights;

      --  Seeking, the contact is looked for as the presses so far predict it, or by doubling; Halving, a step met
      --  something and the span the contact lies in is halved from its near end; Pressing, the span is no longer
      --  than Least and the tool stands where the descent began, for the press.
      type Phase is (Seeking, Halving, Pressing);
      State  : Phase := Seeking;
      --  Along the way down from where the descent began: the deepest the tool went and met nothing, where it stands
      --  while seeking and halving; and, once a step met something, the farthest the contact can be.
      Top    : Real := 0.0;
      Bottom : Real := 0.0;
      Moved  : Boolean := False;   --  a step was reached: the tool no longer stands where the descent began
      At_Stop : Boolean := False;  --  the span's far end is where the step that met something stopped the tool
      Done   : Boolean := False;

      --  How many steps doubling from Step takes to cover Over.
      function Doublings (Over, Step : Real) return Natural is
        (Natural (Real'Ceiling (Ada.Numerics.Long_Elementary_Functions.Log (Over / Step + 1.0, 2.0))));

      --  Back, to where the descent began for the press when the span is no longer than Least, else to the near end
      --  of the span, where the step that met something began. Done when the arm does not get there.
      procedure Go_Back is
         Got_There : Boolean;
         To_Start  : constant Boolean := Bottom - Top <= Least;
      begin
         Back (To_Start, Got_There);
         Steps.Backs := Steps.Backs + 1;
         if not Got_There then
            Steps.Unbacked := True;
            Done := True;
         elsif To_Start then
            State := Pressing;
            Moved := False;
         else
            State := Halving;
         end if;
      end Go_Back;
   begin
      Steps := (others => <>);
      Seen := Above.all;
      --  The arm followed the aim and the hand did not go down with it: the hand lies on what it met, and the descent
      --  is the press, a bound or a contact, not spent.
      if Seen.Stalled then
         Steps.Stalled := True;
         return;
      end if;
      --  The eye's room above the surface is as far as the surface can be: its measured height and Z of its sigma, not
      --  that less them. A tip below the eye meets the surface first, and the press is made where it does; the step that
      --  meets it ends the descent, not where the estimate puts the surface. Less the margin, h22 (servo seed 3, 10-17)
      --  stopped every press with the eye 4.5 of its 14.8 above the table (its sigma 1.5), the jaws' tips 4 cm above it;
      --  at the height itself h23 still stopped short, its lowering in the fit's units some 15 per cent longer than the
      --  arm's in the world: nothing was met, no tip measured. Safety is the least of the driver's concerns (owner,
      --  10-05): the eye may meet the surface too, and that is the event.
      if Known (Seen.Eye) then
         declare
            Room : constant Real :=
              Seen.Eye.Value + Threshold (Scalar_Gate (Seen.Eye.Degrees_Of_Freedom)) * Seen.Eye.Sigma;
         begin
            if Room >= Least then
               --  Doubling from Least covers the room, the last step cut to it and the one
               --  that finds none left; a prediction adds its band and the doubling past it.
               Allowed := Doublings (Room, Least) + 2;
               if Known (Seen.Tip) then
                  Allowed := Allowed + 2 * Natural (Real'Ceiling (Threshold (Scalar_Gate (Seen.Tip.Degrees_Of_Freedom)))) + 1
                    + Doublings (Room, Real'Max (Seen.Tip.Sigma, Least));
               end if;
            end if;
         end;
      end if;
      loop
         declare
            type Kind is (Doubling, Banded, Blind, Probe, Press);
            G   : constant Estimate := Seen.Tip;
            By  : Real := Least;
            How : Kind := Blind;
         begin
            case State is
               when Halving =>
                  --  A surface that does not give stops the tool where the contact is, as the table did every
                  --  press of A79 and A90 that reached it: the first step goes to Least short of that, and when it
                  --  meets nothing the span is no longer than Least. When it meets something, the surface gave, and
                  --  the span is halved.
                  if At_Stop then
                     By := Bottom - Least - Top;
                  else
                     By := (Bottom - Top) / 2.0;
                  end if;
                  At_Stop := False;
                  How := Probe;
               when Pressing =>
                  --  To Least past the near end of the span: past its far end, which a push to it exactly would only
                  --  reach when the contact is there, and no more than Least past the contact.
                  By := Top + Least;
                  How := Press;
               when Seeking =>
                  --  Steps that were reached and went on without the tool meeting anything are not lowering it:
                  --  they end, on the schedule, not on the clock (A17: 6000 beats of steps the hand slid under).
                  if Steps.Fast + Steps.Band + Steps.Blind >= Allowed then
                     Steps.Spent := True;
                     exit;
                  end if;
                  if Known (G) then
                     declare
                        Spread : constant Real := Threshold (Scalar_Gate (G.Degrees_Of_Freedom)) * G.Sigma;
                        --  How far the tip may go before it is within Z sigma of the
                        --  predicted contact.
                        Room : constant Real := G.Value - Spread;
                     begin
                        if Room >= Least then
                           --  The prediction says where the contact is: the first push goes to Z sigma above it, in
                           --  one (the steps used to double from Least over it, thirteen pushes of ten beats to cover
                           --  A22's 4.5 units, and a stop of the arm's own in the air showed at the sixth of them).
                           By := Room;
                           How := Doubling;
                        elsif G.Value + Spread > 0.0 then
                           By := Real'Max (G.Sigma, Least);
                           How := Banded;
                        else
                           --  The band is behind and nothing was met: the contact is not
                           --  where it was predicted, as when the press that fixed the tip
                           --  was stopped above the surface by something under it, and the
                           --  prediction is only a bound. Steps of the band's size would
                           --  creep to the table (A17: 6000 beats, then the hand slid
                           --  along it, each small push reached): they double again.
                           if Past = 0.0 then
                              Driver.Log.Line
                                (Driver.Log.Robot,
                                 "hand: the tip has gone past the contact its presses predict and the band of its sigma"
                                 & " about it, and met nothing (it is predicted" & Driver.Log.Image (G.Value, 4)
                                 & " above it now): the steps double again");
                           end if;
                           Past := Real'Max (2.0 * Past, Real'Max (G.Sigma, Least));
                           By := Past;
                        end if;
                     end;
                  else
                     --  Nothing predicts the contact: doubling until a step meets something.
                     By := Fast;
                     Fast := 2.0 * Fast;
                  end if;
                  --  No step takes the eye past as far as the surface can be (the room, as above), and the last step
                  --  goes to it though it is less than Least: seeking the surface asks only whether a step meets it, which
                  --  the step's own push tells (Blocked), not how far the tool went. Ending when less than Least was left,
                  --  h27 (servo seed 3, 10-17), its Least 10 to 11.5 where its aims' fit placed the tool loosely, made one
                  --  step of each press and stopped with 9.5 of the room left and the jaws' tips short of the table.
                  if Known (Seen.Eye) then
                     declare
                        Room : constant Real :=
                          Seen.Eye.Value + Threshold (Scalar_Gate (Seen.Eye.Degrees_Of_Freedom)) * Seen.Eye.Sigma;
                     begin
                        if Room <= 0.0 or else (Room < Least and then Cut) then
                           Steps.Spent := True;
                           exit;
                        elsif By > Room then
                           By := Room;
                           Steps.Capped := Steps.Capped + 1;
                           Cut := True;
                        else
                           Cut := False;
                        end if;
                     end;
                  end if;
            end case;
            case How is
               when Doubling => Steps.Fast := Steps.Fast + 1;
               when Banded   => Steps.Band := Steps.Band + 1;
               when Blind    => Steps.Blind := Steps.Blind + 1;
               when Probe    => Steps.Probes := Steps.Probes + 1;
               when Press    => Steps.Pressed := Steps.Pressed + 1;
            end case;
            Lower (By, Outcome);
            exit when Outcome = Ended;
            Seen := Above.all;
            --  A step the arm followed and the hand did not go down with met something all the same: the hand lies on
            --  what it met (A17's third press).
            if Outcome = Reached and then Seen.Stalled then
               Outcome := Met;
               Steps.Stalled := True;
            else
               Steps.Stalled := False;
            end if;
            if Outcome = Reached then
               if State = Pressing then
                  --  Nothing where the span said: what stopped the step that bounded it is not there now, and the
                  --  contact is looked for again from here.
                  Top := Bottom;
                  State := Seeking;
               else
                  Top := Top + By;
               end if;
               Moved := True;
               if State = Halving and then Bottom - Top <= Least then
                  Go_Back;
               end if;
            elsif State = Pressing
              or else (not Moved and then By <= Least)
            then
               --  The press: a push from where the descent began, ending no more than Least past where nothing was met.
               exit;
            else
               --  Something met further than Least past where nothing was: the span the contact lies in, its far end
               --  where the tool stopped when that is nearer (it went past the contact, not short of it), and back.
               Bottom := Top + By;
               At_Stop := not Steps.Stalled and then Seen.Down > Top and then Seen.Down < Bottom;
               if At_Stop then
                  Bottom := Seen.Down;
               end if;
               Steps.Stalled := False;
               Go_Back;
            end if;
            exit when Done;
         end;
      end loop;
   end Descend;

   function Sweepable (H : Hands; M : Model; G : Group_Id) return Boolean is
     (H.Data /= null
      and then (for some P of H.Data.Pairs => P.Group = G and then Still_A_Pair (P, M)));

   procedure Measure (H : in out Hands; M : in out Model) is separate;
   --  The decider (driver-robot-hand-measure.adb).

   function Found (H : Hands; Id : Hand_Id) return Hand_Record is (H.Data.Found (Id));

   function Hand_Count (H : Hands) return Natural is (if H.Data = null then 0 else Natural (H.Data.Found.Length));

   function Exists (H : Hands; Id : Hand_Id) return Boolean is (H.Data /= null and then Id <= H.Data.Found.Last_Index);
   function Closer_Group (H : Hands; Id : Hand_Id) return Group_Id'Base is (Found (H, Id).Group);
   function Arm_Of (H : Hands; Id : Hand_Id) return Arm_Id is (Found (H, Id).Arm);
   function Lobe_Count (H : Hands; Id : Hand_Id) return Positive is (Natural (Found (H, Id).Lobes.Length));
   function Presses_Kept (H : Hands; Id : Hand_Id) return Natural is (Driver.Robot.Hand.Tips.Pressed (Found (H, Id).Book));
   function Own_Eye (H : Hands; Id : Hand_Id) return Eye_Id is (Found (H, Id).Eye);

   function Tip_Pixel (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Driver.Images.Pixel is
     (Found (H, Id).Lobes (Lobe).Sights (At_Opening).Pixel);

   function Tip_Sight (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Ray_Estimate is
     (Found (H, Id).Lobes (Lobe).Sights (At_Opening).Ray);

   function Tip_Footprint (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Real is
      Distance : constant Estimate := Driver.Robot.Hand.Tips.Distance (Found (H, Id).Book, Lobe, At_Opening);
   begin
      return (if Known (Distance) then abs Distance.Value * Found (H, Id).Lobes (Lobe).Sights (At_Opening).Spread
              else 0.0);
   end Tip_Footprint;

   function Closer_Reading (H : Hands; Id : Hand_Id; At_Opening : Opening) return Real_Array is
     (Found (H, Id).Readings (At_Opening).Element);

   function Fraction (H : Hands; M : Model; Id : Hand_Id; O : Observation) return Estimate is
      G : constant Group_Id'Base := Closer_Group (H, Id);
   begin
      --  The hand an arm's tool is has no closer: nothing of it closes.
      if G = 0 or else Natural (G) > Natural (O.Readings.Length) or else not Driver.Observations.Has_Reading (O, G) then
         return Unknown;
      end if;
      declare
         Now    : constant Real_Array := O.Readings (G);
         Open   : constant Real_Array := Closer_Reading (H, Id, Hand.Open);
         Closed : constant Real_Array := Closer_Reading (H, Id, Closed_Empty);
         Way    : Real := 0.0;
         Along  : Real := 0.0;
         Var    : Real := 0.0;
      begin
         if Now'Length /= Open'Length or else Open'Length /= Closed'Length then
            return Unknown;
         end if;
         for K in Open'Range loop
            Way := Way + (Closed (K) - Open (K)) ** 2;
         end loop;
         if not (Way > 0.0) then
            return Unknown;
         end if;
         for K in Open'Range loop
            declare
               Channel : constant Positive := K - Open'First + 1;
               D       : constant Real := Closed (K) - Open (K);
            begin
               Along := Along + (Now (K - Open'First + Now'First) - Open (K)) * D;
               Var := Var + (D / Way) ** 2 * Reading_Noise (M, G, Channel) ** 2;
            end;
         end loop;
         return (Value => Along / Way, Sigma => Sqrt (Var), Degrees_Of_Freedom => 0);
      end;
   end Fraction;

   function Lobe_Width (H : Hands; Id : Hand_Id; Lobe : Positive) return Estimate is
     (Found (H, Id).Lobes (Lobe).Size.Width);

   function Lobe_Thickness (H : Hands; Id : Hand_Id; Lobe : Positive) return Estimate is
     (Found (H, Id).Lobes (Lobe).Size.Thickness);

   function Lobe_Face (H : Hands; Id : Hand_Id; Lobe : Positive) return Estimate is
     (Found (H, Id).Lobes (Lobe).Size.Face);

   function Grip_Depth (H : Hands; Id : Hand_Id) return Estimate is (Found (H, Id).Depth);

   function Grip_Axis (H : Hands; Id : Hand_Id) return Direction_Estimate is (Found (H, Id).Axis);

   --  Where along its line of sight a tip is comes only from presses, and
   --  none is measured yet: these report an unknown estimate.

   function Tip_In_Tool
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded)
     return Point_Estimate
   is
      R : constant Hand_Record := Found (H, Id);
   begin
      --  A lobe no channel moves has one tip, pressed at the open setting, whichever is asked.
      return Driver.Robot.Hand.Tips.Tip (R.Book, Lobe, (if R.Lobes (Lobe).Channel = 0 then Open else At_Opening), Kind);
   end Tip_In_Tool;

   function Tip_Beat
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Driver.Clock.Beat is
     (Driver.Robot.Hand.Tips.Beat (Found (H, Id).Book, Lobe, At_Opening, Kind));

   function Slides (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening) return Slide_Readings is
      Kept : constant Driver.Robot.Hand.Tips.Press_Slides :=
        Driver.Robot.Hand.Tips.Slides_Of (Found (H, Id).Book, Lobe, At_Opening);
   begin
      return [for K of Kept => (Beat           => K.Beat,
                                Contact        => K.Contact,
                                Tip_Rests      => K.Agrees,
                                Known          => K.Slid.Known,
                                Pixels         => K.Slid.Pixels,
                                Pixels_Sigma   => K.Slid.Pixels_Sigma,
                                Fraction       => K.Slid.Fraction,
                                Fraction_Sigma => K.Slid.Fraction_Sigma)];
   end Slides;

   function Tip_Confirmed
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Boolean is
     (Driver.Robot.Hand.Tips.Confirmed (Found (H, Id).Book, Lobe, At_Opening, Kind));

   function Tip_Tested
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Boolean is
     (Driver.Robot.Hand.Tips.Tested (Found (H, Id).Book, Lobe, At_Opening, Kind));

   function Tip_Across
     (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; Kind : Tip_Kind := Loaded) return Real_Array is
     (Driver.Robot.Hand.Tips.Across (Found (H, Id).Book, Lobe, At_Opening, Kind));

   function Press_Direction (H : Hands; Id : Hand_Id; Lobe : Positive; At_Opening : Opening)
     return Direction_Estimate is
     (Driver.Robot.Hand.Tips.Direction (Found (H, Id).Book, Lobe, At_Opening));

   function In_World (H : Hands; M : Model; Id : Hand_Id; O : Observation; Tip : Point_Estimate) return Point_Estimate is
     (Driver.Robot.Hand.Frames.Into_World
        (Tool_Pose (M, Arm_Of (H, Id), O), Tool_In_Arm (M, Arm_Of (H, Id), O), Arm_Unit (M, Arm_Of (H, Id)), Tip));
   --  A point of the hand's tool frame, which is in the arm's unit, in the world.

   function Tip (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; At_Opening : Opening; O : Observation)
     return Point_Estimate is
     (In_World (H, M, Id, O, Tip_In_Tool (H, Id, Lobe, At_Opening)));

   function Opening_Reading (R : Hand_Record; Lobe : Positive; Which : Opening) return Real is
     (if R.Lobes (Lobe).Channel = 0 then 0.0
      else R.Readings (Which).Element (R.Readings (Which).Element'First + R.Lobes (Lobe).Channel - 1));
   --  The lobe's channel's reading at the opening.

   function Settings (H : Hands; Id : Hand_Id; Lobe : Positive) return Positive is
     (2 + Natural (Found (H, Id).Lobes (Lobe).Stops.Length));

   function Setting_Sight_At (H : Hands; Id : Hand_Id; Lobe : Positive; S : Positive) return Setting_Sight is
      R    : constant Hand_Record := Found (H, Id);
      L    : Lobe_Record renames R.Lobes (Lobe);
      Last : constant Positive := 2 + Natural (L.Stops.Length);
   begin
      if S = 1 or else S = Last then
         declare
            Which : constant Opening := (if S = 1 then Open else Closed_Empty);
         begin
            return (Reading => Opening_Reading (R, Lobe, Which), Known => L.Sights (Which).Known, Ray => L.Sights (Which).Ray);
         end;
      end if;
      declare
         --  The stops are kept by the reading, ascending; the settings run from the open opening to the closed one.
         Ascending : constant Boolean := Opening_Reading (R, Lobe, Closed_Empty) > Opening_Reading (R, Lobe, Open);
         K         : constant Positive := (if Ascending then S - 1 else Last - S);
         Stop      : Stop_Sight renames L.Stops (K);
      begin
         return (Reading => Stop.Reading, Known => Stop.Sight.Known, Ray => Stop.Sight.Ray);
      end;
   end Setting_Sight_At;

   function Tip_Between
     (Opened, Shut : Point_Estimate; Open_At, Closed_At, Reading : Real; Seen : Setting_Sights) return Point_Estimate
   is
      Unknown_Tip : Point_Estimate;
      Span : constant Real := Closed_At - Open_At;
      function Fraction_Of (R : Real) return Real is ((R - Open_At) / Span);
      --  The straight path between the openings' tips, at a share of the way.
      function Straight (G : Real) return Point_Estimate is
        (Mean       => Opened.Mean + G * (Shut.Mean - Opened.Mean),
         Covariance => ((1.0 - G) ** 2) * Opened.Covariance + (G ** 2) * Shut.Covariance);
      --  The tip at a seen setting: the point of its line of sight nearest the straight path's, the pull as error.
      function Pulled (S : Setting_Sight) return Point_Estimate;
      function Pulled (S : Setting_Sight) return Point_Estimate is
         X     : constant Point_Estimate := Straight (Fraction_Of (S.Reading));
         E     : constant Vec3 := S.Ray.Origin.Mean;
         D     : constant Vec3 := S.Ray.Direction.Unit_Vector;
         On    : constant Vec3 := E + Real'((X.Mean - E) * D) * D;
         Pull  : constant Vec3 := X.Mean - On;
      begin
         return (Mean => On, Covariance => X.Covariance + Real'(Pull * Pull) * Identity3);
      end Pulled;
      F : Real;
      --  The settings on either side of the reading, the openings among them, as shares of the way and tips.
      Lower_F : Real := 0.0;
      Upper_F : Real := 1.0;
      Lower, Upper : Point_Estimate;
   begin
      if not Known (Opened) or else not Known (Shut) or else Span = 0.0 then
         return Unknown_Tip;
      end if;
      F := Fraction_Of (Reading);
      if F <= 0.0 or else F >= 1.0 then
         return Straight (F);
      end if;
      Lower := Opened;
      Upper := Shut;
      for S of Seen loop
         if S.Known and then S.Ray.Direction.Sigma < Real'Last then
            declare
               G : constant Real := Fraction_Of (S.Reading);
            begin
               if G > 0.0 and then G < 1.0 then
                  if G <= F and then G >= Lower_F then
                     Lower_F := G;
                     Lower := Pulled (S);
                  end if;
                  if G >= F and then G <= Upper_F then
                     Upper_F := G;
                     Upper := Pulled (S);
                  end if;
               end if;
            end;
         end if;
      end loop;
      if Upper_F = Lower_F then
         return Lower;
      end if;
      declare
         T : constant Real := (F - Lower_F) / (Upper_F - Lower_F);
      begin
         return (Mean       => Lower.Mean + T * (Upper.Mean - Lower.Mean),
                 Covariance => ((1.0 - T) ** 2) * Lower.Covariance + (T ** 2) * Upper.Covariance);
      end;
   end Tip_Between;

   function Tip_Now_In_Tool (H : Hands; Id : Hand_Id; Lobe : Positive; O : Observation) return Point_Estimate;
   --  The tip at the closer reading of O, between the two measured openings
   --  along the settings the eye saw the lobe at (Tip_Between).

   function Tip_Along (H : Hands; Id : Hand_Id; Lobe : Positive; Reading : Real) return Point_Estimate is
      R       : constant Hand_Record := Found (H, Id);
      Opened  : constant Point_Estimate := Tip_In_Tool (H, Id, Lobe, Open);
      Shut    : constant Point_Estimate := Tip_In_Tool (H, Id, Lobe, Closed_Empty);
      Unknown_Tip : Point_Estimate;
   begin
      if not Known (Opened) or else not Known (Shut) then
         return Unknown_Tip;
      end if;
      declare
         Seen : Setting_Sights (1 .. Natural (R.Lobes (Lobe).Stops.Length));
      begin
         for K in Seen'Range loop
            Seen (K) := (Reading => R.Lobes (Lobe).Stops (K).Reading,
                         Known   => R.Lobes (Lobe).Stops (K).Sight.Known,
                         Ray     => R.Lobes (Lobe).Stops (K).Sight.Ray);
         end loop;
         return Tip_Between (Opened, Shut, Opening_Reading (R, Lobe, Open), Opening_Reading (R, Lobe, Closed_Empty),
                             Reading, Seen);
      end;
   end Tip_Along;

   function Tip_Now_In_Tool (H : Hands; Id : Hand_Id; Lobe : Positive; O : Observation) return Point_Estimate is
      R : constant Hand_Record := Found (H, Id);
      C : constant Natural := R.Lobes (Lobe).Channel;
      Unknown_Tip : Point_Estimate;
   begin
      --  A lobe no channel moves is where it is at any reading.
      if C = 0 then
         return Tip_Along (H, Id, Lobe, 0.0);
      end if;
      if not Driver.Observations.Has_Reading (O, R.Group) then
         return Unknown_Tip;
      end if;
      return Tip_Along (H, Id, Lobe, O.Readings.Element (R.Group) (O.Readings.Element (R.Group)'First + C - 1));
   end Tip_Now_In_Tool;

   function Tip_Now (H : Hands; M : Model; Id : Hand_Id; Lobe : Positive; O : Observation) return Point_Estimate is
     (In_World (H, M, Id, O, Tip_Now_In_Tool (H, Id, Lobe, O)));

   function Grip_Centre (H : Hands; M : Model; Id : Hand_Id; O : Observation) return Point_Estimate is
      Sum   : Vec3 := Zero3;
      Total : Mat3 := [others => [others => 0.0]];
      Count : constant Positive := Lobe_Count (H, Id);
   begin
      for L in 1 .. Count loop
         declare
            T : constant Point_Estimate := Tip_Now_In_Tool (H, Id, L, O);
         begin
            if not Known (T) then
               return In_World (H, M, Id, O, T);
            end if;
            Sum := Sum + T.Mean;
            Total := Total + T.Covariance;
         end;
      end loop;
      --  The lobes' tips at this opening, averaged; each tip was fitted from
      --  its own presses.
      return In_World (H, M, Id, O, (Mean => (1.0 / Real (Count)) * Sum, Covariance => (1.0 / Real (Count) ** 2) * Total));
   end Grip_Centre;

   function Describe (H : Hands) return String is
      Text : Unbounded_String;
   begin
      --  Every closer of an arm's own eye that no hand is made of says why not,
      --  channel by channel.
      if H.Data /= null then
         for P of H.Data.Pairs loop
            if not (for some R of H.Data.Found => R.Group = P.Group) then
               Append (Text, "closer group" & P.Group'Image & " on arm" & P.Arm'Image & " in eye" & P.Eye'Image
                       & ", its own: no hand;");
               for C in 1 .. Sweeps.Channels (P.Sweep) loop
                  Append (Text, " channel" & C'Image & ": " & Sweeps.Account (P.Sweep, C) & ";");
               end loop;
               Append (Text, ASCII.LF);
            end if;
         end loop;
      end if;
      for Id in 1 .. Hand_Id'Base (Hand_Count (H)) loop
         declare
            R      : constant Hand_Record := Found (H, Id);
            Known_Tips, Checked_Tips, Sought : Natural := 0;
         begin
            Append (Text, "hand" & Id'Image
                    & (if R.Group > 0 then ": closer group" & R.Group'Image else ": the tool, no closer,") & " on arm"
                    & R.Arm'Image & ", eye" & R.Eye'Image & "," & R.Lobes.Length'Image & " lobes;");
            for L in R.Lobes.First_Index .. R.Lobes.Last_Index loop
               Append (Text, " channel" & R.Lobes (L).Channel'Image & " tip open " & Image (R.Lobes (L).Sights (Open).Pixel)
                       & " closed " & Image (R.Lobes (L).Sights (Closed_Empty).Pixel) & ";");
               for Which in Opening loop
                  if R.Lobes (L).Sights (Which).Known then
                     Sought := Sought + 1;
                     Known_Tips := Known_Tips + Boolean'Pos (Known (Driver.Robot.Hand.Tips.Tip (R.Book, L, Which)));
                     Checked_Tips := Checked_Tips + Boolean'Pos (Driver.Robot.Hand.Tips.Confirmed (R.Book, L, Which));
                  end if;
               end loop;
            end loop;
            Append (Text, " presses:" & Driver.Robot.Hand.Tips.Pressed (R.Book)'Image & " kept, tips measured:"
                    & Known_Tips'Image & " of" & Sought'Image & " (" & Natural'Image (Checked_Tips)
                    & " confirmed by a second press); sizes: " & Sizes_Text (R) & ASCII.LF);
         end;
      end loop;
      return To_String (Text);
   end Describe;

end Driver.Robot.Hand;
