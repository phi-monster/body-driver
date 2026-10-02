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

   procedure One_Press is
      W     : Watcher;
      Found : Boolean;
      Press : Event;
      Count : Natural := 0;
      Got   : Event;
      type Beat_Kind is record
         Blocked, Still : Boolean;
         Height         : Real;
      end record;
      --  Still above the table, moving down, blocked while pushing in (the
      --  tool sinks), let go and resting a little higher, then pushed again
      --  and held blocked without ever letting go.
      Stream : constant array (1 .. 12) of Beat_Kind :=
        [(False, True, 0.20), (False, True, 0.20), (False, False, 0.15), (False, False, 0.11),
         (True, False, 0.098), (True, True, 0.097), (True, True, 0.097), (False, True, 0.099),
         (False, True, 0.099), (True, False, 0.098), (True, True, 0.097), (True, True, 0.097)];
   begin
      for B in Stream'Range loop
         Observe (W, Driver.Clock.Beat (B), Stream (B).Blocked, Stream (B).Still, Pose_At (Stream (B).Height),
                  [1 => 0.04], Found, Press);
         if Found then
            Count := Count + 1;
            Got := Press;
         end if;
      end loop;
      Check (Count = 1, "the stream gave" & Count'Image & " presses");
      if Count = 1 then
         Check (Got.Beat = 8 and then Got.Tool.Pose.Translation (3) = 0.099,
                "the press is not the pose at rest after the push let go");
         --  Down in the world is +x in the tool, which points down.
         Check (Got.Approach.Sigma < Real'Last and then Got.Approach.Unit_Vector (1) > 0.999,
                "the press direction is not the way the tool moved");
         Check (Got.Closer.Element (1) = 0.04, "the closer readings at the press were not kept");
      end if;
   end One_Press;

   procedure Unmoved_Press_Has_No_Direction is
      W     : Watcher;
      Found : Boolean;
      Press : Event;
   begin
      Observe (W, 1, False, True, Pose_At (0.1), [1 => 0.0], Found, Press);
      Observe (W, 2, True, True, Pose_At (0.1), [1 => 0.0], Found, Press);
      Observe (W, 3, False, True, Pose_At (0.1), [1 => 0.0], Found, Press);
      Check (Found and then Press.Approach.Sigma = Real'Last, "a block without a move was given a direction");
   end Unmoved_Press_Has_No_Direction;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.presses.stream", "a press is read at the wrong beat or with the wrong direction",
                             One_Press'Access);
      Driver.Tests.Register ("hand.presses.unmoved", "a press that moved nothing is given a direction",
                             Unmoved_Press_Has_No_Direction'Access);
   end Register;

end Driver.Robot.Hand.Presses.Tests;
