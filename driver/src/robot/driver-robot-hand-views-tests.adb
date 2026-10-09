with Driver.Bytes;
with Driver.Tests;

package body Driver.Robot.Hand.Views.Tests is

   use Driver.Tests;
   use type Driver.Clock.Beat;
   use type Driver.Bytes.Offset;

   Side : constant := 16;

   --  What the eye sees of a hand: a block of a third of the picture, at the level the
   --  hand's opening gives it, on a background that does not change. A level for the
   --  whole picture would be a change of the lighting, and not a change of anything.
   function Grey (Level : Natural) return Driver.Images.Image is
      Data : Driver.Bytes.Byte_Array (1 .. 3 * Side * Side);
   begin
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            for Channel in 1 .. 3 loop
               Data (Driver.Bytes.Offset (3 * (Row * Side + Column) + Channel)) :=
                 Driver.Bytes.Byte (if Column < Side / 3 then Level else 100);
            end loop;
         end loop;
      end loop;
      return Driver.Images.Create (Side, Side, Data);
   end Grey;

   function At_Beat (B : Driver.Clock.Beat) return Observation is ((Beat => B, others => <>));
   --  The tracker reads only the beat; its readings and image are passed beside it.

   function Exact (Before, After : Real_Array) return Boolean is (Before /= After);
   --  The rest of a body whose readings repeat exactly moved when they changed.

   --  A12's far arm: its reading's noise at rest 2.4e-16, the smallest step
   --  of it its eye sees 1e-6, and, uncommanded, it creeps 1.2e-12 a beat.
   --  By its noise alone that creep is motion; by the body's one test of
   --  motion (Driver.Robot.Channels.Visible, which the hand passes), a watched
   --  channel moves only by a step its eye can see, and it is not.
   Arm_Noise : constant Real := 2.4e-16;
   Arm_Seen  : constant Real := 1.0e-6;
   Arm_Creep : constant Real := 1.2e-12;

   function By_Noise (Before, After : Real_Array) return Boolean is
     (for some I in Before'Range => Significant (After (I) - Before (I), Arm_Noise));

   function By_Eye (Before, After : Real_Array) return Boolean is
     (for some I in Before'Range => abs (After (I) - Before (I)) >= Arm_Seen);

   procedure Creeping_Arm is
      --  The closer pushed once and the eye still, as A12 after its first
      --  push, the far arm creeping all the while: by its noise the arm
      --  moved every beat and no view ever had two frames; by what an eye
      --  can see, the view forms at its second frame and holds.
      By_Noise_T : Tracker := Start (Side, Side, Closer_Noise => [1 => 0.0]);
      By_Eye_T   : Tracker := Start (Side, Side, Closer_Noise => [1 => 0.0]);
      Arm        : Real := 0.0;
      Formed_By_Noise, Formed_By_Eye : Natural := 0;
   begin
      for B in 0 .. 49 loop
         Arm := Arm + Arm_Creep;
         Observe (By_Noise_T, At_Beat (Driver.Clock.Beat (B)), True, [1 => 0.99998], [1 => Arm], Grey (10),
                  By_Noise'Access);
         Observe (By_Eye_T, At_Beat (Driver.Clock.Beat (B)), True, [1 => 0.99998], [1 => Arm], Grey (10),
                  By_Eye'Access);
         Formed_By_Noise := Formed_By_Noise + Boolean'Pos (Gathered (By_Noise_T));
         Formed_By_Eye := Formed_By_Eye + Boolean'Pos (Gathered (By_Eye_T));
      end loop;
      Check (Formed_By_Noise = 0, "the creep by the noise alone let a view form: the stand-in for A12 is wrong");
      Check (Formed_By_Eye = 49, "a view of a still eye did not form at its second frame and hold while a far arm"
             & " crept by what no eye sees: it was formed" & Formed_By_Eye'Image & " of 49 beats");
   end Creeping_Arm;

   procedure Ends_Of_A_Sweep is
      T : Tracker := Start (Side, Side, Closer_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
      procedure Beat (Still : Boolean; Closer : Real; Level : Natural; Arm : Real := 0.0) is
      begin
         Observe (T, At_Beat (B), Still, [1 => Closer], [Arm, 0.0], Grey (Level), Exact'Access);
         B := B + 1;
      end Beat;
   begin
      --  Open and still for three frames, moving, half closed, moving,
      --  closed for two frames, moving, open again for two.
      Beat (True, 1.0, 10); Beat (True, 1.0, 10); Beat (True, 1.0, 10);
      Beat (False, 0.8, 20);
      Beat (True, 0.5, 30); Beat (True, 0.5, 30); Beat (True, 0.5, 30);
      Beat (False, 0.2, 40);
      Check (not Has_Ends (T, 1) or else Reading (Low_End (T, 1), 1) = 0.5, "a view counted before it ended");
      Beat (True, 0.0, 50); Beat (True, 0.0, 50);
      Beat (False, 0.5, 60);
      Beat (True, 1.0, 10); Beat (True, 1.0, 10);
      Beat (False, 1.0, 10);
      Check (Has_Ends (T, 1), "a sweep from open to closed gave no ends");
      if Has_Ends (T, 1) then
         Check (Reading (Low_End (T, 1), 1) = 0.0 and then Reading (High_End (T, 1), 1) = 1.0,
                "the ends are not the lowest and highest readings");
         Check (Driver.Pixels.Frames (High_End (T, 1).Frames) = 3 and then High_End (T, 1).From = 0,
                "the open end is not the view with the most frames");
         Check (Driver.Pixels.Frames (Low_End (T, 1).Frames) = 2 and then Low_End (T, 1).From = 8,
                "the closed end is not its own view");
      end if;
      --  The arm moves, the closer at an end, as when the arm carries the closer's eye to another pose: the ends
      --  seen before stay, the robot's own pixels being where they were in an eye the arm carries.
      Beat (True, 0.0, 70, Arm => 0.3); Beat (True, 0.0, 70, Arm => 0.3);
      Beat (False, 0.0, 70, Arm => 0.3);
      Check (Has_Ends (T, 1) and then Low_End (T, 1).From = 8 and then High_End (T, 1).From = 0,
             "the ends were not kept across a move of the arm with the closer at an end");
      --  The closer moves at the new pose: what was seen before is no longer comparable, and the ends start again.
      Beat (True, 0.5, 80, Arm => 0.3); Beat (True, 0.5, 80, Arm => 0.3);
      Beat (False, 0.5, 80, Arm => 0.3);
      Check (not Has_Ends (T, 1), "ends were kept across a move of the arm and of the closer");
   end Ends_Of_A_Sweep;

   --  The block at a level the closer's reading does not touch, on a background lit
   --  (1 - Reading) times one to three levels more, a level either way at each pixel.
   function Lit (Reading : Real) return Driver.Images.Image is
      Data : Driver.Bytes.Byte_Array (1 .. 3 * Side * Side);
   begin
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            for Channel in 1 .. 3 loop
               Data (Driver.Bytes.Offset (3 * (Row * Side + Column) + Channel)) :=
                 Driver.Bytes.Byte
                   (if Column < Side / 3 then 10
                    else Natural (Real'Rounding (100.0 + (1.0 - Reading)
                                                  * (1.0 + 2.0 * Real (Column) / Real (Side - 1)
                                                     + Real ((Column * 7 + Row * 13) mod 3) - 1.0))));
            end loop;
         end loop;
      end loop;
      return Driver.Images.Create (Side, Side, Data);
   end Lit;

   procedure Light_Is_Not_Travel is
      --  The closer swept from open to closed and back, the eye seeing only the
      --  light change with it: the views of its two ends do not differ, and its
      --  push moves nothing here.
      T : Tracker := Start (Side, Side, Closer_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
      procedure Beat (Still : Boolean; Closer : Real) is
      begin
         Observe (T, At_Beat (B), Still, [1 => Closer], [1 => 0.0], Lit (Closer), Exact'Access);
         B := B + 1;
      end Beat;
   begin
      Beat (True, 1.0); Beat (True, 1.0); Beat (True, 1.0);
      Beat (False, 0.5);
      Beat (True, 0.0); Beat (True, 0.0); Beat (True, 0.0);
      Beat (False, 0.5);
      Beat (True, 1.0); Beat (True, 1.0);
      Beat (False, 1.0);
      Check (not Has_Ends (T, 1), "lighting that moves with the closer was taken for the travel of a hand");
      Check (Unseen_Travel (T, 1), "a travel the eye saw only the light change with was not said to move nothing");
   end Light_Is_Not_Travel;

   procedure Single_Frames_Are_Not_Ends is
      T : Tracker := Start (Side, Side, Closer_Noise => [1 => 0.0]);
   begin
      Observe (T, At_Beat (0), True, [1 => 1.0], [1 => 0.0], Grey (10), Exact'Access);
      Observe (T, At_Beat (1), False, [1 => 0.5], [1 => 0.0], Grey (20), Exact'Access);
      Observe (T, At_Beat (2), True, [1 => 0.0], [1 => 0.0], Grey (30), Exact'Access);
      Observe (T, At_Beat (3), False, [1 => 0.5], [1 => 0.0], Grey (20), Exact'Access);
      Check (not Has_Ends (T, 1), "views of one frame were taken as ends");
   end Single_Frames_Are_Not_Ends;

   procedure Noise_Hides_A_Small_Step is
      --  A reading noise of 0.1: steps of 0.05 are the same view, 1.0 is not.
      T : Tracker := Start (Side, Side, Closer_Noise => [1 => 0.1]);
   begin
      Observe (T, At_Beat (0), True, [1 => 1.0], [1 => 0.0], Grey (10), Exact'Access);
      Observe (T, At_Beat (1), True, [1 => 1.05], [1 => 0.0], Grey (10), Exact'Access);
      Observe (T, At_Beat (2), True, [1 => 0.95], [1 => 0.0], Grey (10), Exact'Access);
      Observe (T, At_Beat (3), False, [1 => 0.5], [1 => 0.0], Grey (20), Exact'Access);
      Observe (T, At_Beat (4), True, [1 => 0.0], [1 => 0.0], Grey (30), Exact'Access);
      Observe (T, At_Beat (5), True, [1 => 0.02], [1 => 0.0], Grey (30), Exact'Access);
      Observe (T, At_Beat (6), False, [1 => 0.5], [1 => 0.0], Grey (20), Exact'Access);
      Check (Has_Ends (T, 1), "noisy readings broke one still view into many");
      if Has_Ends (T, 1) then
         Check (Driver.Pixels.Frames (High_End (T, 1).Frames) = 3, "the noisy open view lost frames");
      end if;
   end Noise_Hides_A_Small_Step;

   procedure Past_The_Travel is
      --  A closer whose reading echoes its command, commanded past its travel:
      --  the reading goes on to -0.5 but the fingers stopped at 0.0. A channel
      --  that moves nothing the eye sees has no travel at all.
      T : Tracker := Start (Side, Side, Closer_Noise => [1 => 0.0]);
      Idle : Tracker := Start (Side, Side, Closer_Noise => [1 => 0.0]);
      B : Driver.Clock.Beat := 0;
      procedure Hold (Reading : Real; Level : Natural) is
      begin
         for I in 1 .. 2 loop
            Observe (T, At_Beat (B), True, [1 => Reading], [1 => 0.0], Grey (Level), Exact'Access);
            Observe (Idle, At_Beat (B), True, [1 => Reading], [1 => 0.0], Grey (10), Exact'Access);
            B := B + 1;
         end loop;
         Observe (T, At_Beat (B), False, [1 => Reading], [1 => 0.0], Grey (Level), Exact'Access);
         Observe (Idle, At_Beat (B), False, [1 => Reading], [1 => 0.0], Grey (10), Exact'Access);
         B := B + 1;
      end Hold;
      procedure Still_At (Reading : Real; Level : Natural) is
      begin
         for I in 1 .. 2 loop
            Observe (T, At_Beat (B), True, [1 => Reading], [1 => 0.0], Grey (Level), Exact'Access);
            B := B + 1;
         end loop;
      end Still_At;
   begin
      Hold (1.0, 10);
      Hold (0.5, 30);
      --  While a view is gathered, a sweep asks whether it extends the travel.
      Still_At (0.0, 50);
      Check (Would_Extend (T, 1, Exact'Access), "a view that shows the fingers further is not taken to extend the travel");
      Hold (0.0, 50);
      Still_At (-0.5, 50);
      Check (not Would_Extend (T, 1, Exact'Access), "a view past the travel is taken to extend it");
      Hold (-0.5, 50);
      Check (Has_Ends (T, 1) and then Reading (Low_End (T, 1), 1) = 0.0 and then Reading (High_End (T, 1), 1) = 1.0,
             "a command past the travel moved the end");
      Check (not Has_Ends (Idle, 1) and then Unseen_Travel (Idle, 1) and then not Unseen_Travel (T, 1),
             "a push the eye never sees was given a travel, or one it sees none");
   end Past_The_Travel;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.views.sweep", "the ends of a channel's travel are not its lowest and highest still views",
                             Ends_Of_A_Sweep'Access);
      Driver.Tests.Register ("hand.views.beyond", "a command past the travel, which the eye shows changing nothing, moves the end",
                             Past_The_Travel'Access);
      Driver.Tests.Register ("hand.views.light", "a closer whose push the eye sees only as lighting is given a travel",
                             Light_Is_Not_Travel'Access);
      Driver.Tests.Register ("hand.views.frames", "views too short to measure their own noise are taken as ends",
                             Single_Frames_Are_Not_Ends'Access);
      Driver.Tests.Register ("hand.views.noise", "reading noise splits one still view", Noise_Hides_A_Small_Step'Access);
      Driver.Tests.Register ("hand.views.creep", "a far arm creeping by what no eye sees keeps a still eye's view from "
                             & "forming (A12)", Creeping_Arm'Access);
   end Register;

end Driver.Robot.Hand.Views.Tests;
