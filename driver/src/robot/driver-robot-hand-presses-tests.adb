with Ada.Numerics;
with Driver.Tests;

package body Driver.Robot.Hand.Presses.Tests is

   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Clock.Beat;

   --  The tool turned so that its own x axis points down, held at a height.
   Turned : constant Mat3 := Exp ([0.0, Ada.Numerics.Pi / 2.0, 0.0]);

   function Pose_At (Height : Real) return Pose_Estimate is
     ((Pose                => (Rotation => Turned, Translation => [0.3, 0.1, Height]),
       Position_Covariance => 1.0e-8 * Identity3,
       Rotation_Covariance => 1.0e-8 * Identity3));

   type Beat_Kind is record
      Blocked, Pushing, Still : Boolean;   --  the verdict on the latest push, that push under way, the arm at rest
      Height                  : Real;
   end record;

   type Stream_Kind is array (Positive range <>) of Beat_Kind;

   --  Every press the stream gives, with the beat and the height of the tool at it.
   type Found_Press is record
      Beat   : Driver.Clock.Beat;
      Height : Real;
      Event  : Presses.Event;
   end record;

   type Found_Array is array (Positive range <>) of Found_Press;

   type Commands_Kind is array (Positive range <>) of Boolean;
   --  The beats at which a new command for the arm takes effect.

   function Run (Stream : Stream_Kind; Commanded : Commands_Kind := [1 .. 0 => False]) return Found_Array is
      W     : Watcher;
      Found : Boolean;
      Press : Event;
      None  : Found_Array (1 .. 0);
      Count : Natural := 0;
      Got   : Found_Array (1 .. Stream'Length);
   begin
      for B in Stream'Range loop
         Observe (W, Driver.Clock.Beat (B), Stream (B).Blocked, Stream (B).Pushing, Stream (B).Still,
                  Pose_At (Stream (B).Height), [1 => Stream (B).Height], [1 => 0.04], Found, Press,
                  Retargeted => B in Commanded'Range and then Commanded (B));
         if Found then
            Count := Count + 1;
            Got (Count) := (Beat => Driver.Clock.Beat (B), Height => Stream (B).Height, Event => Press);
         end if;
      end loop;
      return (if Count = 0 then None else Got (1 .. Count));
   end Run;

   procedure One_Press is
      --  Still above the table, moving down, blocked while pushing in (the
      --  tool sinks), let go and resting a little higher, then pushed again
      --  and held blocked without ever letting go.
      Stream : constant Stream_Kind :=
        [(False, False, True, 0.20), (False, False, True, 0.20), (False, True, False, 0.15), (False, True, False, 0.11),
         (True, False, False, 0.098), (True, False, True, 0.097), (True, False, True, 0.097), (False, False, True, 0.099),
         (False, False, True, 0.099), (True, False, False, 0.098), (True, False, True, 0.097), (True, False, True, 0.097)];
      Got : constant Found_Array := Run (Stream);
   begin
      Check (Got'Length = 1, "the stream gave" & Got'Length'Image & " presses");
      if Got'Length = 1 then
         Check (Got (1).Event.Beat = 8 and then Got (1).Event.Tool.Pose.Translation (3) = 0.099,
                "the press is not the pose at rest after the push let go");
         --  Down in the world is +x in the tool, which points down.
         Check (Got (1).Event.Approach.Sigma < Real'Last and then Got (1).Event.Approach.Unit_Vector (1) > 0.999,
                "the press direction is not the way the tool moved");
         Check (Got (1).Event.Closer.Element (1) = 0.04, "the closer readings at the press were not kept");
         Check (Got (1).Event.Arm.Element (1) = 0.099, "the arm's readings at the press were not kept");
      end if;
   end One_Press;

   procedure Unmoved_Press_Has_No_Direction is
      W     : Watcher;
      Found : Boolean;
      Press : Event;
   begin
      Observe (W, 1, False, False, True, Pose_At (0.1), [1 => 0.1], [1 => 0.0], Found, Press);
      Observe (W, 2, True, False, True, Pose_At (0.1), [1 => 0.1], [1 => 0.0], Found, Press);
      Observe (W, 3, False, False, True, Pose_At (0.1), [1 => 0.1], [1 => 0.0], Found, Press);
      Check (Found and then Press.Approach.Sigma = Real'Last, "a block without a move was given a direction");
   end Unmoved_Press_Has_No_Direction;

   procedure Let_Go_Judged_Blocked is
      --  The press as A16's arm made it (the arm group's episodes 267, 268
      --  and 269): the push that met the table ends at rest, Blocked; the
      --  let-go, a push that asks nothing, lasts 83 beats while the arm eases
      --  back, and ends at rest judged Blocked itself; the retreat follows and
      --  ends at rest above the table. The press is the rest after the
      --  let-go: not the stop (loaded), not the retreat's end (the aim), and
      --  the let-go's verdict does not begin another press. A second press
      --  after it, whose let-go is judged as it should be, is found as well.
      Stream : constant Stream_Kind :=
        [1  => (False, False, True, 0.200),   --  free, at rest above the table
         2  => (False, False, True, 0.200),
         3  => (False, True, False, 0.150),   --  the descent
         4  => (False, True, False, 0.110),
         5  => (True, False, True, 0.0980),   --  the push ended at rest, blocked: the stop
         6  => (True, False, True, 0.0980),
         7  => (True, False, True, 0.0980),
         8  => (False, True, False, 0.0982),  --  the let-go begins: the verdict falls
         9  => (False, True, False, 0.0985),
         10 => (False, True, False, 0.0988),
         11 => (False, True, False, 0.0989),
         12 => (True, False, True, 0.0990),    --  the let-go ended at rest, judged blocked: the press
         13 => (True, False, False, 0.0990),   --  still easing (the arm group is not at rest)
         14 => (True, False, False, 0.0990),
         15 => (False, True, False, 0.150),    --  the retreat
         16 => (False, True, False, 0.190),
         17 => (False, False, True, 0.200),    --  at rest above the table: free again
         18 => (False, True, False, 0.150),    --  a second press
         19 => (True, False, True, 0.0980),
         20 => (False, True, False, 0.0982),
         21 => (False, False, True, 0.0990)];
      Got : constant Found_Array := Run (Stream);
   begin
      Check (Got'Length = 2, "the stream gave" & Got'Length'Image & " presses, not two");
      if Got'Length = 2 then
         Check (Got (1).Beat = 12 and then Got (1).Height = 0.0990,
                "the first press is at beat" & Got (1).Beat'Image & ", height" & Got (1).Height'Image
                & ", not at the rest after the let-go");
         Check (Got (2).Beat = 21 and then Got (2).Height = 0.0990,
                "the second press is at beat" & Got (2).Beat'Image & ", height" & Got (2).Height'Image
                & ", not at the rest after its let-go");
         Check (Got (1).Event.Approach.Sigma < Real'Last and then Got (1).Event.Approach.Unit_Vector (1) > 0.999,
                "the press direction is not the way the tool moved before the block");
      end if;
   end Let_Go_Judged_Blocked;

   procedure Let_Go_Answered_Late is
      --  A body that answers the let-go two beats late is at rest when the
      --  push begins and for two beats after: the press is the rest at the
      --  end of that push, where the arm has eased back, not the stop it
      --  stood at while the push was under way.
      Stream : constant Stream_Kind :=
        [1 => (False, False, True, 0.200),
         2 => (False, True, False, 0.150),
         3 => (True, False, True, 0.0980),    --  the stop
         4 => (False, True, True, 0.0980),    --  the let-go begins; the arm has not answered
         5 => (False, True, True, 0.0980),
         6 => (False, True, False, 0.0985),   --  easing back
         7 => (False, True, False, 0.0988),
         8 => (False, False, True, 0.0990)];  --  the let-go ended at rest
      Got : constant Found_Array := Run (Stream);
   begin
      Check (Got'Length = 1 and then Got (1).Beat = 8,
             "a body that answers the let-go late gave" & Got'Length'Image & " presses, the first at beat"
             & (if Got'Length > 0 then Got (1).Beat'Image else " none"));
   end Let_Go_Answered_Late;

   procedure Hold_Begins_No_Push is
      --  A35's first press: the push that met the table ended blocked while the arm was still easing back; the hold at
      --  the readings the block left asked the arm nothing the step tracker sees, so no push began and the verdict
      --  stood; the arm eased back a few beats more, came to rest, and the retreat followed. The press is the rest
      --  after the hold, where the hand is on the table, not the rest at the aim after the retreat.
      Stream : constant Stream_Kind :=
        [1  => (False, False, True, 0.200),
         2  => (False, True, False, 0.150),     --  the descent
         3  => (False, True, False, 0.110),
         4  => (True, False, False, 0.0980),    --  the push ended blocked, the arm still easing
         5  => (True, False, False, 0.0982),
         6  => (True, False, False, 0.0985),    --  the hold takes effect: no push begins, the verdict stands
         7  => (True, False, False, 0.0988),
         8  => (True, False, True, 0.0990),     --  at rest: the press
         9  => (True, False, True, 0.0990),
         10 => (False, True, False, 0.150),     --  the retreat
         11 => (False, False, True, 0.200)];    --  at rest at the aim
      Commanded : constant Commands_Kind (1 .. 11) := [6 | 10 => True, others => False];
      Got : constant Found_Array := Run (Stream, Commanded);
   begin
      Check (Got'Length = 1 and then Got (1).Beat = 8 and then Got (1).Height = 0.0990,
             "a hold that began no push gave" & Got'Length'Image & " presses, the first at beat"
             & (if Got'Length > 0 then Got (1).Beat'Image & ", height" & Got (1).Height'Image else " none"));
   end Hold_Begins_No_Push;

   procedure Retreat_Before_The_Rest_Is_No_Press is
      --  The same, the retreat sent before the arm came to rest: the rest that follows is the aim's. No press; and
      --  the watcher is free again at that rest, so that the press after it is found.
      Stream : constant Stream_Kind :=
        [1  => (False, False, True, 0.200),
         2  => (False, True, False, 0.150),
         3  => (False, True, False, 0.110),
         4  => (True, False, False, 0.0980),
         5  => (True, False, False, 0.0982),
         6  => (True, False, False, 0.0985),    --  the hold takes effect
         7  => (True, False, False, 0.0988),    --  the arm is still easing back
         8  => (False, True, False, 0.120),     --  the retreat takes effect before any rest
         9  => (False, True, False, 0.170),
         10 => (False, False, True, 0.200),     --  at rest at the aim: not a press
         11 => (False, True, False, 0.150),     --  a second press
         12 => (True, False, False, 0.0980),
         13 => (True, False, False, 0.0985),    --  its hold takes effect
         14 => (True, False, True, 0.0990)];    --  and rests: the press
      Commanded : constant Commands_Kind (1 .. 14) := [6 | 8 | 13 => True, others => False];
      Got : constant Found_Array := Run (Stream, Commanded);
   begin
      Check (Got'Length = 1 and then Got (1).Beat = 14,
             "a retreat before the rest gave" & Got'Length'Image & " presses, the first at beat"
             & (if Got'Length > 0 then Got (1).Beat'Image & ", height" & Got (1).Height'Image else " none"));
   end Retreat_Before_The_Rest_Is_No_Press;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.presses.hold", "a hold that began no push is not the let-go, or the rest after the "
                             & "retreat is taken for the press",
                             Hold_Begins_No_Push'Access);
      Driver.Tests.Register ("hand.presses.retreat", "a retreat sent before the hand rested gives a press at the aim, or "
                             & "the watcher is not free after it", Retreat_Before_The_Rest_Is_No_Press'Access);
      Driver.Tests.Register ("hand.presses.stream", "a press is read at the wrong beat or with the wrong direction",
                             One_Press'Access);
      Driver.Tests.Register ("hand.presses.unmoved", "a press that moved nothing is given a direction",
                             Unmoved_Press_Has_No_Direction'Access);
      Driver.Tests.Register ("hand.presses.letgo",
                             "a press is read under the push, at the retreat's end, or begun again by the let-go's verdict",
                             Let_Go_Judged_Blocked'Access);
      Driver.Tests.Register ("hand.presses.late",
                             "a body that answers the let-go late is read at the stop, not after the let-go",
                             Let_Go_Answered_Late'Access);
   end Register;

end Driver.Robot.Hand.Presses.Tests;
