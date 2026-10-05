with Ada.Exceptions;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Tests;

package body Driver.Robot.Hand.Sweep.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;
   use type Driver.Clock.Beat;
   use type Driver.Bytes.Offset;

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

   function Frame (R : Real; Scale : Positive := 1) return Driver.Images.Image is
      Data : Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Scale * W * Scale * H));
   begin
      for Row in 0 .. Scale * H - 1 loop
         for C in 0 .. Scale * W - 1 loop
            declare
               --  A textured table, so every background pixel has its own value.
               Table : constant Natural :=
                 Natural (128.0 + 60.0 * Sin (Real (C) * 0.37) * Cos (Real (Row) * 0.23) + Real ((C * 7 + Row * 13) mod 19));
               L : constant Driver.Bytes.Byte :=
                 Driver.Bytes.Byte (if On_Finger (C, Row, R, Scale) > 0 then 20 else Table);
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

   --  The matcher's answers for points of the view at reading From, looked
   --  for in the view at reading To.
   function Answers (Points : Driver.Instrument.Point_Array; From, To : Real; Scale : Positive := 1)
     return Driver.Instrument.Answer_Array is
   begin
      return Result : Driver.Instrument.Answer_Array (Points'Range) do
         for K in Points'Range loop
            declare
               P : constant Driver.Images.Pixel := Points (K);
               C : constant Natural := Natural (Real'Floor (P.U));
               R : constant Natural := Natural (Real'Floor (P.V));
               F : constant Natural := On_Finger (C, R, From, Scale);
            begin
               if F > 0 then
                  Result (K) := (Found => True, Certainty => 1.0, Back => P,
                                 To => (U => P.U + Real (Left_Edge (F, To, Scale) - Left_Edge (F, From, Scale)),
                                        V => P.V));
               elsif On_Finger (C, R, To, Scale) > 0 then
                  --  Covered in the other view: nowhere to go, no way back.
                  Result (K) := (Found => True, Certainty => 0.1, To => (U => P.U + 7.0, V => P.V),
                                 Back => (U => P.U + 11.0, V => P.V + 3.0));
               else
                  Result (K) := (Found => True, Certainty => 1.0, To => P, Back => P);
               end if;
            end;
         end loop;
      end return;
   end Answers;

   procedure Two_Fingers_Swept is
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
      procedure Hold (R : Real; Frames : Positive) is
      begin
         for I in 1 .. Frames loop
            Observe (S, At_Beat (B), True, [1 => R], [1 => 0.0], Frame (R), Exact'Access);
            B := B + 1;
         end loop;
      end Hold;
      procedure Move (R : Real) is
      begin
         Observe (S, At_Beat (B), False, [1 => R], [1 => 0.0], Frame (R), Exact'Access);
         B := B + 1;
      end Move;
   begin
      --  Open, then pushed closed in two steps, then opened again.
      Hold (1.0, 3);
      Move (0.7);
      Hold (0.5, 2);
      Move (0.2);
      Hold (0.0, 3);
      Move (0.5);
      Hold (1.0, 2);
      Move (1.0);
      Check (Wants_Correspondences (S, 1), "a full sweep did not ask for correspondences");
      if Wants_Correspondences (S, 1) then
         declare
            Points : constant Driver.Instrument.Point_Array := Query_Points (S, 1);
            Low  : constant Real := Views.Reading (Low_End (S, 1), 1);
            High : constant Real := Views.Reading (High_End (S, 1), 1);
         begin
            Check (Low = 0.0 and then High = 1.0, "the ends are not closed and open");
            --  The box of the change spans both fingers' travel from the top row down.
            Check (Points'Length = (149 - 10 + 1) * (H - 40), "the query box is not the change's box:"
                   & Natural'Image (Points'Length));
            Asked (S, 1);
            Check (Status (S, 1) = Requested and then not Wants_Correspondences (S, 1), "asking was not recorded");
            Answer (S, 1, Points, Answers (Points, Low, High), Answers (Points, High, Low), Driver.Images.Create (W, H));
         end;
         Check (Status (S, 1) = Measured, "the answers gave no lobes");
         if Status (S, 1) = Measured then
            Check (Natural (Lobes_Of (S, 1).Length) = 2, "two fingers gave"
                   & Natural'Image (Natural (Lobes_Of (S, 1).Length)) & " lobes");
            Check (Closing_Known (S, 1) and then not Closed_End_Is_High (S, 1),
                   "the closed end is not the low reading");
         end if;
      end if;
   end Two_Fingers_Swept;

   procedure Swept_In_A_Task is
      --  The hand's estimate runs inside the decider's task, whose stack is
      --  GNAT's default: the closer swept open to closed and back in a VGA
      --  eye the fingers fill much of, the change between its ends asked of
      --  the matcher pixel by pixel both ways round, and the lobes found from
      --  the answers. Every per-pixel quantity of that is megabytes.
      Scale    : constant := 4;
      Readings : constant Real_Array := [1.0, 0.7, 0.5, 0.2, 0.0];
      type Frame_Array is array (Readings'Range) of Driver.Images.Image;
      Frames   : constant Frame_Array := [for I in Readings'Range => Frame (Readings (I), Scale)];
      S        : State := Start (Scale * W, Scale * H, Channels => 1, Closer_Noise => [1 => 0.0]);
      Done     : Boolean := False with Atomic;
      Asked_At : Natural := 0;
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
                  Observe (S, At_Beat (B), True, [1 => R], [1 => 0.0], Frame_At (R), Exact'Access);
                  B := B + 1;
               end loop;
            end Hold;
            procedure Move (R : Real) is
            begin
               Observe (S, At_Beat (B), False, [1 => R], [1 => 0.0], Frame_At (R), Exact'Access);
               B := B + 1;
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
            if Wants_Correspondences (S, 1) then
               declare
                  Points : constant Driver.Instrument.Point_Array := Query_Points (S, 1);
                  Low    : constant Real := Views.Reading (Low_End (S, 1), 1);
                  High   : constant Real := Views.Reading (High_End (S, 1), 1);
               begin
                  Asked_At := Points'Length;
                  Asked (S, 1);
                  Answer (S, 1, Points, Answers (Points, Low, High, Scale), Answers (Points, High, Low, Scale),
                          Driver.Images.Create (Scale * W, Scale * H));
               end;
               if Status (S, 1) = Measured then
                  Found := Natural (Lobes_Of (S, 1).Length);
               end if;
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
      Check (not Done or else Found = 2, "two fingers in a VGA view gave" & Found'Image & " lobes, from"
             & Asked_At'Image & " pixels asked");
   end Swept_In_A_Task;

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
      Result.Ends := Wants_Correspondences (S, 1);
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
      Check (Status (S, 1) = Nothing_Moves and then not Wants_Correspondences (S, 1),
             "a push that changes nothing asked the matcher");
   end Nothing_Seen;

   procedure Refused (Lasting : Boolean) is
      --  A sweep asked, refused, and swept again with new ends.
      S : State := Start (W, H, Channels => 1, Closer_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
      Rest_At : Real := 0.0;   --  the rest of the body, which the boot moves between sweeps
      procedure Hold (R : Real; Frames : Positive) is
      begin
         for I in 1 .. Frames loop
            Observe (S, At_Beat (B), True, [1 => R], [1 => Rest_At], Frame (R), Exact'Access);
            B := B + 1;
         end loop;
      end Hold;
      procedure Move (R : Real) is
      begin
         Observe (S, At_Beat (B), False, [1 => R], [1 => Rest_At], Frame (R), Exact'Access);
         B := B + 1;
      end Move;
      procedure Sweep is
      begin
         Hold (1.0, 3);
         Move (0.5);
         Hold (0.0, 3);
         Move (0.5);
         Hold (1.0, 3);
         Move (1.0);
      end Sweep;
   begin
      Sweep;
      Check (Wants_Correspondences (S, 1), "a full sweep did not ask for correspondences");
      if Wants_Correspondences (S, 1) then
         Asked (S, 1);
         Refuse (S, 1, Lasting, "no address was given for the instrument service");
         --  The arm moved, and the closer was swept again there: new ends.
         Rest_At := 0.5;
         Sweep;
         if Lasting then
            Check (Status (S, 1) = Unanswerable and then not Wants_Correspondences (S, 1)
                   and then Refusal (S) = "no address was given for the instrument service",
                   "an instrument that can never answer is asked again for new ends, or the reason is lost");
         else
            Check (Wants_Correspondences (S, 1) and then Refusal (S) = "",
                   "new ends are not asked for after a refusal that may pass");
         end if;
      end if;
   end Refused;

   procedure Refused_For_Good is
   begin
      Refused (Lasting => True);
   end Refused_For_Good;

   procedure Refused_For_Now is
   begin
      Refused (Lasting => False);
   end Refused_For_Now;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.sweep.lasting",
                             "an instrument that can never answer is asked again whenever the closer's ends are new",
                             Refused_For_Good'Access);
      Driver.Tests.Register ("hand.sweep.transient", "new ends are not asked for after a refusal that may pass",
                             Refused_For_Now'Access);
      Driver.Tests.Register ("hand.sweep.two", "a closer swept open to closed does not yield its lobes and closed end",
                             Two_Fingers_Swept'Access);
      Driver.Tests.Register ("hand.sweep.task", "the sweep's estimate fails in a task with the default stack, as the "
                             & "decider's does", Swept_In_A_Task'Access);
      Driver.Tests.Register ("hand.sweep.nothing", "a push that changes nothing in the eye is sent to the matcher",
                             Nothing_Seen'Access);
      Driver.Tests.Register ("hand.sweep.blind", "a sweep from a step its views cannot see stops at its first push "
                             & "with no ends (A11), or pushes unseen without bound", Below_The_Views'Access);
   end Register;

end Driver.Robot.Hand.Sweep.Tests;
