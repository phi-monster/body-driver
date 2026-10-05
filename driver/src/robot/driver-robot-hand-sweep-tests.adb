with Ada.Exceptions;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Fixed;
with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Tests;

package body Driver.Robot.Hand.Sweep.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;
   use type Driver.Clock.Beat;
   use type Driver.Bytes.Offset;
   use type Driver.Robot.Hand.Lobes.Placing;

   W : constant := 160;
   H : constant := 120;

   function At_Beat (B : Driver.Clock.Beat) return Observation is ((Beat => B, others => <>));
   --  The sweep reads only the beat; its readings and image are passed beside it.

   function Exact (Before, After : Real_Array) return Boolean is (Before /= After);
   --  The rest of a body whose readings repeat exactly moved when they changed.

   --  Two dark fingers enter from the bottom border; at closer reading R
   --  (1 open, 0 closed) each has moved (1 - R) * 45 columns inwards. A view
   --  Scale times as wide and high shows the same, Scale times as large.
   function Left_Edge (Finger : Positive; R : Real; Scale : Positive := 1) return Integer is
     (Scale * (if Finger = 1 then 10 + Integer (45.0 * (1.0 - R)) else 130 - Integer (45.0 * (1.0 - R))));

   function On_Finger (C, Row : Natural; R : Real; Scale : Positive := 1) return Natural is
   begin
      for F in 1 .. 2 loop
         if Row >= Scale * 40
           and then Integer (C) in Left_Edge (F, R, Scale) .. Left_Edge (F, R, Scale) + Scale * 20 - 1
         then
            return F;
         end if;
      end loop;
      return 0;
   end On_Finger;

   --  The eye of an arm. Pose: the arm's, which slides the table across the
   --  picture while the fingers, that go with the eye, stay where the closer
   --  puts them. Light: the renderer's lighting moves with the fingers: at
   --  reading R every pixel of the table is lit (1 - R) times one to three
   --  levels more, more towards the right and by a level either way at each
   --  pixel. Fingers False leaves the table alone with that lighting. Wide:
   --  two fifths of the columns are lifted by forty levels (1 - R) and the
   --  next three tenths lowered by as much, a change of far more than half
   --  the picture. Floating: the fingers are cut off from the bottom border.
   function Frame
     (R        : Real;
      Pose     : Natural := 0;
      Scale    : Positive := 1;
      Light    : Boolean := False;
      Fingers  : Boolean := True;
      Wide     : Boolean := False;
      Floating : Boolean := False) return Driver.Images.Image
   is
      Data : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Scale * W * Scale * H));
   begin
      for Row in 0 .. Scale * H - 1 loop
         for C in 0 .. Scale * W - 1 loop
            declare
               --  A textured table, so every background pixel has its own value.
               Seen_C : constant Integer := C + 5 * Pose;
               Seen_R : constant Integer := Row + 3 * Pose;
               Table : constant Natural :=
                 Natural (128.0 + 60.0 * Sin (Real (Seen_C) * 0.37) * Cos (Real (Seen_R) * 0.23)
                          + Real ((Seen_C * 7 + Seen_R * 13) mod 19));
               Lit   : constant Real :=
                 (if Light
                  then (1.0 - R) * (1.0 + 2.0 * Real (C) / Real (Scale * W - 1) + Real ((C * 7 + Row * 13) mod 3) - 1.0)
                  else 0.0)
                 + (if Wide and then C < 4 * Scale * W / 10 then 40.0 * (1.0 - R)
                    elsif Wide and then C < 7 * Scale * W / 10 then -40.0 * (1.0 - R)
                    else 0.0);
               Cut_Off : constant Boolean := Floating and then Row > Scale * (H - 10);
               L : constant Driver.Bytes.Byte :=
                 Driver.Bytes.Byte (if Fingers and then not Cut_Off and then On_Finger (C, Row, R, Scale) > 0 then 20
                                    else Natural (Real'Max (0.0, Real'Min (255.0, Real'Rounding (Real (Table) + Lit)))));
               K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Row * Scale * W + C));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Scale * W, Scale * H, Data);
   end Frame;

   --  How a scene is drawn, for the choreography below.
   type Scene is record
      Light, Fingers, Wide, Floating : Boolean := False;
      Moving_Arm                     : Boolean := True;   --  the arm's poses move the eye
   end record;

   Plain : constant Scene := (Fingers => True, Moving_Arm => True, others => False);

   --  The closer sits open, at reading 1, while the arm goes through
   --  Boot_Poses poses, three still frames each (the body's boot); then the
   --  closer is swept at one more pose: open, closed, open, its readings
   --  moving between. The rest of the body is the arm's pose number.
   Boot_Poses : constant := 12;

   procedure Observe_Frame
     (S     : in out State;
      B     : in out Driver.Clock.Beat;
      Still : Boolean;
      R     : Real;
      Pose  : Natural;
      Looks : Scene)
   is
   begin
      Observe (S, At_Beat (B), Still, [1 => R], [1 => Real (Pose)],
               Frame (R, (if Looks.Moving_Arm then Pose else 0), 1, Looks.Light, Looks.Fingers, Looks.Wide, Looks.Floating),
               Exact'Access);
      B := B + 1;
   end Observe_Frame;

   procedure Boot (S : in out State; B : in out Driver.Clock.Beat; Looks : Scene) is
   begin
      for P in 0 .. Boot_Poses - 1 loop
         for I in 1 .. 3 loop
            Observe_Frame (S, B, True, 1.0, P, Looks);
         end loop;
      end loop;
   end Boot;

   procedure Sweep (S : in out State; B : in out Driver.Clock.Beat; Looks : Scene) is
      Pose : constant := Boot_Poses;
      procedure Hold (R : Real; Frames : Positive) is
      begin
         for I in 1 .. Frames loop
            Observe_Frame (S, B, True, R, Pose, Looks);
         end loop;
      end Hold;
      procedure Move (R : Real) is
      begin
         Observe_Frame (S, B, False, R, Pose, Looks);
      end Move;
   begin
      Hold (1.0, 3);
      Move (0.7);
      Hold (0.5, 2);
      Move (0.2);
      Hold (0.0, 3);
      Move (0.5);
      Hold (1.0, 2);
      Move (1.0);
   end Sweep;

   function Swept_Scene (Looks : Scene) return State is
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
   begin
      Boot (S, B, Looks);
      Sweep (S, B, Looks);
      return S;
   end Swept_Scene;

   function Mentions (Text, Part : String) return Boolean is (Ada.Strings.Fixed.Index (Text, Part) > 0);

   procedure Two_Fingers_Swept is
      S : constant State := Swept_Scene (Plain);
   begin
      Check (Has_Ends (S, 1), "a full sweep gave no ends");
      if Has_Ends (S, 1) then
         Check (Views.Reading (Low_End (S, 1), 1) = 0.0 and then Views.Reading (High_End (S, 1), 1) = 1.0,
                "the ends are not closed and open");
      end if;
      Check (Status (S, 1) = Measured, "the sweep gave no lobes: " & Account (S, 1));
      if Status (S, 1) = Measured then
         Check (Natural (Lobes_Of (S, 1).Length) = 2, "two fingers gave"
                & Natural'Image (Natural (Lobes_Of (S, 1).Length)) & " lobes");
         Check (Closing_Known (S, 1) and then not Closed_End_Is_High (S, 1), "the closed end is not the low reading");
         for L of Lobes_Of (S, 1) loop
            Check (L.Tip_Known_Here and then L.Tip_Known_There and then abs (L.Tip_Here.V - 40.5) < 1.0e-9
                   and then abs (L.Tip_There.V - 40.5) < 1.0e-9,
                   "a lobe's tip is not its finger's top");
         end loop;
         --  Everything that changed is the fingers' swept area, which is two
         --  fingers' two places of 20 columns by 80 rows.
         Check (Located_Of (S, 1).Changed = 4 * 20 * 80, "the pixels that changed are not the fingers' two places:"
                & Natural'Image (Located_Of (S, 1).Changed));
         Check (Located_Of (S, 1).Unassigned = 0, "a pixel that changed was given to neither end:"
                & Natural'Image (Located_Of (S, 1).Unassigned));
      end if;
   end Two_Fingers_Swept;

   procedure Swept_In_A_Task is
      --  The hand's estimate runs inside the decider's task, whose stack is
      --  GNAT's default: the closer swept open to closed and back in a VGA
      --  eye the fingers fill much of, the arm's poses kept as per-pixel
      --  statistics, the change between its ends compared and its lobes found.
      --  Every per-pixel quantity of that is megabytes.
      Scale    : constant := 4;
      Readings : constant Real_Array := [1.0, 0.7, 0.5, 0.2, 0.0];
      type Frame_Array is array (Readings'Range) of Driver.Images.Image;
      Frames   : constant Frame_Array := [for I in Readings'Range => Frame (Readings (I), Boot_Poses, Scale)];
      type Pose_Array is array (0 .. Boot_Poses - 1) of Driver.Images.Image;
      Poses    : constant Pose_Array := [for P in Pose_Array'Range => Frame (1.0, P, Scale)];
      S        : State := Start (Scale * W, Scale * H, Channels => 1, Closer_Noise => [1 => 0.0]);
      Done     : Boolean := False with Atomic;
      Found    : Natural := 0;
      Failure  : Ada.Strings.Unbounded.Unbounded_String;

      function Frame_At (R : Real) return Driver.Images.Image is
      begin
         for I in Readings'Range loop
            if Readings (I) = R then
               return Frames (I);
            end if;
         end loop;
         raise Program_Error with "no frame drawn at reading" & R'Image;
      end Frame_At;
   begin
      declare
         task Decider;
         task body Decider is
            B : Driver.Clock.Beat := 0;
            procedure Hold (R : Real; Count : Positive) is
            begin
               for I in 1 .. Count loop
                  Observe (S, At_Beat (B), True, [1 => R], [1 => Real (Boot_Poses)], Frame_At (R), Exact'Access);
                  B := B + 1;
               end loop;
            end Hold;
            procedure Move (R : Real) is
            begin
               Observe (S, At_Beat (B), False, [1 => R], [1 => Real (Boot_Poses)], Frame_At (R), Exact'Access);
               B := B + 1;
            end Move;
         begin
            for P in Pose_Array'Range loop
               for I in 1 .. 3 loop
                  Observe (S, At_Beat (B), True, [1 => 1.0], [1 => Real (P)], Poses (P), Exact'Access);
                  B := B + 1;
               end loop;
            end loop;
            Hold (1.0, 3);
            Move (0.7);
            Hold (0.5, 2);
            Move (0.2);
            Hold (0.0, 3);
            Move (0.5);
            Hold (1.0, 2);
            Move (1.0);
            if Status (S, 1) = Measured then
               Found := Natural (Lobes_Of (S, 1).Length);
            end if;
            Done := True;
         exception
            when E : others =>
               Failure := Ada.Strings.Unbounded.To_Unbounded_String (Ada.Exceptions.Exception_Information (E));
         end Decider;
      begin
         null;
      end;
      Check (Done, "the sweep's estimate failed in a task with the default stack: "
             & Ada.Strings.Unbounded.To_String (Failure));
      Check (not Done or else Found = 2, "two fingers in a VGA view gave" & Found'Image & " lobes: " & Account (S, 1));
   end Swept_In_A_Task;

   procedure Lit_By_The_Fingers is
      --  The renderer's lighting moves with the fingers: at the closed end
      --  every pixel of the table is one to three levels brighter, a level
      --  either way at each. That is not a change of anything. A14's two ends
      --  differed so at 85 % of the pixels.
      S : constant State := Swept_Scene ((Light => True, Fingers => True, Moving_Arm => True, others => False));
   begin
      Check (Status (S, 1) = Measured and then Natural (Lobes_Of (S, 1).Length) = 2
             and then Closing_Known (S, 1) and then not Closed_End_Is_High (S, 1),
             "two fingers in moving light were not two lobes closed at the low reading: " & Account (S, 1));
      --  The fingers' two places, and not the lit table.
      Check (Located_Of (S, 1).Changed < 4 * 20 * 80 + 4 * 20 * 80 / 10,
             "the pixels that changed were many more than the fingers' two places:" & Natural'Image (Located_Of (S, 1).Changed));
   end Lit_By_The_Fingers;

   procedure Only_The_Light_Moves is
      --  Nothing but the lighting differs between the ends.
      S : constant State := Swept_Scene ((Light => True, Fingers => False, Moving_Arm => True, others => False));
   begin
      Check (Status (S, 1) = Nothing_Moves, "lighting that moves with the closer, and nothing else, was taken for a hand: "
             & Account (S, 1));
   end Only_The_Light_Moves;

   procedure More_Than_Half_Moves is
      --  Seven tenths of the picture change between the ends, in two ways:
      --  what moved cannot be told from what did not.
      S : constant State := Swept_Scene ((Wide => True, Fingers => False, Moving_Arm => True, others => False));
   begin
      Check (Status (S, 1) = Everything_Moves and then Mentions (Account (S, 1), "half of this eye's picture"),
             "a picture changed over most of it was not said to be: " & Account (S, 1));
   end More_Than_Half_Moves;

   procedure Without_Poses is
      --  The arm's poses did not move the eye (the frames repeat while the
      --  rest of the body's readings change), so nothing tells the fingers
      --  from the table.
      S : constant State := Swept_Scene ((Fingers => True, Moving_Arm => False, others => False));
   begin
      Check (Status (S, 1) = Unplaced and then Located_Of (S, 1).How = Driver.Robot.Hand.Lobes.Unseparated,
             "poses that did not move the eye told fingers from table: " & Account (S, 1));
   end Without_Poses;

   procedure Never_Seen_From_Poses is
      --  The closer is swept without the arm having moved at its starting
      --  reading: one pose.
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
   begin
      Sweep (S, B, Plain);
      Check (Status (S, 1) = Unlocated and then Mentions (Account (S, 1), "has not moved the eye against its surroundings"),
             "a sweep with no poses of the arm at its ends said: " & Account (S, 1));
      --  The arm then goes through poses at the open reading, and the same ends
      --  are placed.
      for P in 0 .. Boot_Poses - 1 loop
         for I in 1 .. 3 loop
            Observe_Frame (S, B, True, 1.0, P, Plain);
         end loop;
      end loop;
      Check (Status (S, 1) = Measured, "ends swept before the poses came were not placed once they did: " & Account (S, 1));
   end Never_Seen_From_Poses;

   procedure Floating_Patch is
      --  A moving patch cut off from the border is attached to nothing.
      S : constant State := Swept_Scene ((Fingers => True, Floating => True, Moving_Arm => True, others => False));
   begin
      Check (Status (S, 1) = Unplaced and then Located_Of (S, 1).How = Driver.Robot.Hand.Lobes.One_Sided,
             "a patch attached to nothing was made a lobe: " & Account (S, 1));
   end Floating_Patch;

   --  What the log says of a channel, at each stage of its sweep: the ends
   --  not seen, nothing moving, everything moving, no poses, poses that tell
   --  nothing, parts attached to nothing, and lobes measured and closed at one
   --  end, or measured but not told which end is closed (with by how much and
   --  against what the lobes' distances changed). A14's hand phase ended on
   --  "closing direction not significant" and then nothing: no hand, no
   --  press, and not a word on whether it was the sweep, the instrument or
   --  the lobes.
   procedure Account_Follows_The_Sweep is
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
   begin
      Observe_Frame (S, B, True, 1.0, Boot_Poses, Plain);
      Check (Mentions (Account (S, 1), "not both seen still"), "a channel with one end says: " & Account (S, 1));
      S := Swept_Scene (Plain);
      Check (Status (S, 1) = Measured and then Mentions (Account (S, 1), "2 lobes, closed at the low reading"),
             "a channel measured says: " & Account (S, 1));
      Check (Mentions (Account (S, 1), "pixels changed") and then Mentions (Account (S, 1), "given to the low end"),
             "a channel measured does not say what changed: " & Account (S, 1));
      Check (Unannounced (S, 1), "ends not yet said are not so");
      Announce (S, 1);
      Check (not Unannounced (S, 1), "ends said are not so");
      S := Swept_Scene ((Wide => True, Fingers => False, Moving_Arm => True, others => False));
      Check (Mentions (Account (S, 1), "half of this eye's picture or more changes"), "everything moving says: " & Account (S, 1));
      S := Swept_Scene ((Fingers => True, Moving_Arm => False, others => False));
      Check (Mentions (Account (S, 1), "do not fall in two groups"), "poses that tell nothing say: " & Account (S, 1));
      S := Swept_Scene ((Fingers => True, Floating => True, Moving_Arm => True, others => False));
      Check (Mentions (Account (S, 1), "larger than the doubt"), "a patch attached to nothing says: " & Account (S, 1));
      S := Swept_Scene (Plain);
      --  The same lobes with nothing to tell which end is closed: how far their distances changed, and the
      --  sigma that was not enough.
      declare
         Per_Channel : Channel_Array := S.Per_Channel.Element;
      begin
         Per_Channel (1).Closing := Driver.Robot.Hand.Lobes.Undecided;
         Per_Channel (1).Change := (Value => 3.0, Sigma => 40.0, Degrees_Of_Freedom => 0);
         S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
         Check (Mentions (Account (S, 1), "closing direction not significant")
                and then Mentions (Account (S, 1), "changed by 3.00"),
                "a channel whose lobes' distances did not tell says: " & Account (S, 1));
         Per_Channel (1).Change := Unknown;
         S.Per_Channel := Channel_Holders.To_Holder (Per_Channel);
         Check (Mentions (Account (S, 1), "nothing to compare"), "a channel with nothing to compare says: " & Account (S, 1));
      end;
   end Account_Follows_The_Sweep;

   procedure Nothing_Seen is
      --  A channel whose push changes nothing this eye sees.
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0]);
   begin
      for I in 1 .. 3 loop
         Observe (S, At_Beat (Driver.Clock.Beat (I)), True, [1 => 1.0], [1 => 0.0], Frame (1.0), Exact'Access);
      end loop;
      Observe (S, At_Beat (4), False, [1 => 0.5], [1 => 0.0], Frame (1.0), Exact'Access);
      for I in 5 .. 7 loop
         Observe (S, At_Beat (Driver.Clock.Beat (I)), True, [1 => 0.0], [1 => 0.0], Frame (1.0), Exact'Access);
      end loop;
      Observe (S, At_Beat (8), False, [1 => 0.5], [1 => 0.0], Frame (1.0), Exact'Access);
      Check (Status (S, 1) = Nothing_Moves, "a push that changes nothing was placed: " & Account (S, 1));
   end Nothing_Seen;

   --  A11's closer: at the upper end of its travel, 0 to 1, swept down and up
   --  from there through the views of its own eye as Hand.Measure sweeps it,
   --  from the step its lock-in sees (group 5's, 1.7e-5 of the travel).
   --
   --  The frames' fingers move 45 columns over the travel and are drawn at
   --  whole columns, so a pair of still views shows a push from a ninetieth
   --  of the travel on, and the view moves a pixel at a forty-fifth: the
   --  lock-in's step is some six hundred times below what the views show.
   --
   --  The rest of the body is an arm whose reading repeats to 1e-16 at rest
   --  and, after each push, settles back over Settling beats from 1e-10,
   --  ten times closer each beat, as A11's arm 1 did from -3.9e-10 against
   --  a noise of 2.4e-16: every beat of that starts the view again. Each push
   --  is followed by one more beat (Hand.Measure's step ends at the closer's
   --  first still beat and holds one more); each asking of Shows is a beat.
   --
   --  Echo: the reading follows any command, past the end too, where the
   --  fingers stay. Once: Shows is asked once a push and takes an unformed
   --  view for nothing new, as A11's sweep did.
   Rest_Noise  : constant Real := 1.0e-16;
   Disturbance : constant Real := 1.0e-10;
   Settling    : constant := 7;

   type Sweep_Outcome is record
      Down, Up    : Natural := 0;   --  pushes each way
      Unseen_Down : Natural := 0;
      Furthest_Up : Real := 0.0;    --  the largest push up asked
      Ends        : Boolean := False;
      Low, High   : Real := 0.0;
   end record;

   --  A11's arm settling after each push, by A11's test of motion then: its
   --  noise alone, every beat of it a move.
   function Arm_Moved (Before, After : Real_Array) return Boolean is
     (for some I in Before'Range => Significant (After (I) - Before (I), Sqrt (2.0) * Rest_Noise));

   function Swept (Step, Pixel : Real; Echo, Once : Boolean) return Sweep_Outcome is
      S       : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0]);
      B       : Driver.Clock.Beat := 0;
      Reading : Real := 1.0;
      Left    : Natural := 0;      --  beats of the arm's settling still to come
      Result  : Sweep_Outcome;
      function Shown_At (R : Real) return Driver.Images.Image is (Frame (Real'Min (1.0, Real'Max (0.0, R))));
      procedure Beat_On (Still : Boolean) is
         Arm : constant Real := (if Left > 0 then Disturbance * 0.1 ** (Settling - Left) else 0.0);
      begin
         Observe (S, At_Beat (B), Still, [1 => Reading], [1 => Arm], Shown_At (Reading), Arm_Moved'Access);
         B := B + 1;
         Left := (if Left > 0 then Left - 1 else 0);
      end Beat_On;
      procedure Go_To (Target : Real; Followed : out Boolean) is
         To : constant Real := (if Echo then Target else Real'Min (1.0, Real'Max (0.0, Target)));
      begin
         Followed := To /= Reading;
         Reading := To;
         Left := (if Followed then Settling else 0);
         Beat_On (Still => False);
         Beat_On (Still => True);
      end Go_To;
      procedure Push (Offset : Real; Followed : out Boolean) is
      begin
         if Offset > 0.0 then
            Result.Furthest_Up := Real'Max (Result.Furthest_Up, Offset);
         end if;
         Go_To (1.0 + Offset, Followed);
      end Push;
      function Shows return Driver.Robot.Hand.Showing is
      begin
         Beat_On (Still => True);
         if not Gathered (S) and then not Once then
            return Driver.Robot.Hand.Not_Yet;
         end if;
         return (if Gathered (S) and then Would_Extend (S, 1, Arm_Moved'Access) then Driver.Robot.Hand.Something_New
                 else Driver.Robot.Hand.Nothing_New);
      end Shows;
      Answered, Formed : Boolean;
      Unseen, Longest  : Natural;
      Back             : Boolean;
      --  Long enough for the arm to settle and two frames after.
      Wait : constant Positive := Settling + 3;
   begin
      for I in 1 .. 3 loop
         Beat_On (Still => True);
      end loop;
      Driver.Robot.Hand.Sweep_Way (-1.0, Step, Pixel, Wait, Push'Access, Shows'Access, Result.Down,
                                   Result.Unseen_Down, Longest, Formed, Answered);
      Go_To (1.0, Back);
      Driver.Robot.Hand.Sweep_Way (1.0, Step, Pixel, Wait, Push'Access, Shows'Access, Result.Up, Unseen, Longest,
                                   Formed, Answered);
      Go_To (1.0, Back);
      for I in 1 .. Settling + 3 loop
         Beat_On (Still => True);
      end loop;
      Result.Ends := Has_Ends (S, 1);
      if Result.Ends then
         Result.Low := Views.Reading (Low_End (S, 1), 1);
         Result.High := Views.Reading (High_End (S, 1), 1);
      end if;
      return Result;
   end Swept;

   procedure Below_The_Views is
      Lockin_Step : constant Real := 1.7e-5;
      Pixel       : constant Real := Driver.Robot.Hand.Seen_By (45.0);
      As_A11      : constant Sweep_Outcome := Swept (Lockin_Step, 0.0, Echo => False, Once => True);
      Unwaited    : constant Sweep_Outcome := Swept (Lockin_Step, Pixel, Echo => False, Once => True);
      Now         : constant Sweep_Outcome := Swept (Lockin_Step, Pixel, Echo => False, Once => False);
      Echoed      : constant Sweep_Outcome := Swept (Lockin_Step, Pixel, Echo => True, Once => False);
   begin
      --  As A11 swept: one push each way, and both ends the view it began in.
      Check (not As_A11.Ends and then As_A11.Down = 1, "A11's sweep found ends, or pushed" & As_A11.Down'Image
             & " times down");
      --  Pushing on unseen is not enough while the views are asked too soon.
      Check (not Unwaited.Ends, "a sweep that asks its views before they form found ends");
      Check (Now.Ends and then Now.Low = 0.0 and then Now.High = 1.0,
             "a sweep from a step its views cannot see, its arm settling after each push, did not find both ends "
             & "of the travel: " & (if Now.Ends then Real'Image (Now.Low) & " to" & Real'Image (Now.High) else "none")
             & " after" & Now.Down'Image & " pushes down," & Now.Unseen_Down'Image & " unseen");
      Check (Now.Unseen_Down > 0 and then Lockin_Step * 2.0 ** (Now.Unseen_Down - 1) <= Pixel,
             "its pushes went on unseen past where its view moves a pixel:" & Now.Unseen_Down'Image & " unseen");
      Check (Now.Up = 1, "up from its upper end the closer was pushed" & Now.Up'Image & " times, not once");
      --  A reading that echoes the command past the end: up, nothing is ever
      --  seen, and the pushes stop within a doubling of a pixel's push.
      Check (Echoed.Furthest_Up >= Pixel and then Echoed.Furthest_Up < 2.0 * Pixel,
             "an echoing closer pushed past its end unseen was asked" & Real'Image (Echoed.Furthest_Up)
             & " past it, a pixel's push being" & Real'Image (Pixel));
   end Below_The_Views;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.sweep.two", "a closer swept open to closed does not yield its lobes and closed end",
                             Two_Fingers_Swept'Access);
      Driver.Tests.Register ("hand.sweep.task", "the sweep's estimate fails in a task with the default stack, as the "
                             & "decider's does", Swept_In_A_Task'Access);
      Driver.Tests.Register ("hand.sweep.nothing", "a push that changes nothing in the eye is placed",
                             Nothing_Seen'Access);
      Driver.Tests.Register ("hand.sweep.lighting", "lighting that moves with the fingers is a change of the whole "
                             & "picture", Lit_By_The_Fingers'Access);
      Driver.Tests.Register ("hand.sweep.lightonly", "a push that changes only the lighting is taken for a hand",
                             Only_The_Light_Moves'Access);
      Driver.Tests.Register ("hand.sweep.crowded", "a picture changed over most of it is taken for a measurement",
                             More_Than_Half_Moves'Access);
      Driver.Tests.Register ("hand.sweep.poses", "poses of the arm that did not move the eye tell fingers from table",
                             Without_Poses'Access);
      Driver.Tests.Register ("hand.sweep.unposed", "ends swept without poses of the arm are never placed once they "
                             & "come", Never_Seen_From_Poses'Access);
      Driver.Tests.Register ("hand.sweep.adrift", "a patch that changed and is attached to nothing is made a lobe",
                             Floating_Patch'Access);
      Driver.Tests.Register ("hand.sweep.account", "a channel's sweep ends, at any stage, without a word of what became "
                             & "of it", Account_Follows_The_Sweep'Access);
      Driver.Tests.Register ("hand.sweep.blind", "a sweep from a step its views cannot see stops at its first push "
                             & "with no ends (A11), or pushes unseen without bound", Below_The_Views'Access);
   end Register;

end Driver.Robot.Hand.Sweep.Tests;
