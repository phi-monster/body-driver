with Driver.Bytes;
with Driver.Tests;

package body Driver.Robot.Hand.Views.Tests is

   use Driver.Tests;
   use type Driver.Clock.Beat;

   function Grey (Level : Natural) return Driver.Images.Image is
     (Driver.Images.Create (4, 4, [1 .. 48 => Driver.Bytes.Byte (Level)]));

   procedure Ends_Of_A_Sweep is
      T : Tracker := Start (4, 4, Closer_Noise => [1 => 0.0], Arm_Noise => [0.0, 0.0]);
      B : Driver.Clock.Beat := 0;
      procedure Beat (Still : Boolean; Closer : Real; Level : Natural; Arm : Real := 0.0) is
      begin
         Observe (T, B, Still, [1 => Closer], [Arm, 0.0], Grey (Level));
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
      --  The arm moves: what was seen before is no longer comparable.
      Beat (True, 0.0, 70, Arm => 0.3); Beat (True, 0.0, 70, Arm => 0.3);
      Beat (False, 0.0, 70, Arm => 0.3);
      Check (not Has_Ends (T, 1), "ends were kept across a move of the arm");
   end Ends_Of_A_Sweep;

   procedure Single_Frames_Are_Not_Ends is
      T : Tracker := Start (4, 4, Closer_Noise => [1 => 0.0], Arm_Noise => [1 => 0.0]);
   begin
      Observe (T, 0, True, [1 => 1.0], [1 => 0.0], Grey (10));
      Observe (T, 1, False, [1 => 0.5], [1 => 0.0], Grey (20));
      Observe (T, 2, True, [1 => 0.0], [1 => 0.0], Grey (30));
      Observe (T, 3, False, [1 => 0.5], [1 => 0.0], Grey (20));
      Check (not Has_Ends (T, 1), "views of one frame were taken as ends");
   end Single_Frames_Are_Not_Ends;

   procedure Noise_Hides_A_Small_Step is
      --  A reading noise of 0.1: steps of 0.05 are the same view, 1.0 is not.
      T : Tracker := Start (4, 4, Closer_Noise => [1 => 0.1], Arm_Noise => [1 => 0.0]);
   begin
      Observe (T, 0, True, [1 => 1.0], [1 => 0.0], Grey (10));
      Observe (T, 1, True, [1 => 1.05], [1 => 0.0], Grey (10));
      Observe (T, 2, True, [1 => 0.95], [1 => 0.0], Grey (10));
      Observe (T, 3, False, [1 => 0.5], [1 => 0.0], Grey (20));
      Observe (T, 4, True, [1 => 0.0], [1 => 0.0], Grey (30));
      Observe (T, 5, True, [1 => 0.02], [1 => 0.0], Grey (30));
      Observe (T, 6, False, [1 => 0.5], [1 => 0.0], Grey (20));
      Check (Has_Ends (T, 1), "noisy readings broke one still view into many");
      if Has_Ends (T, 1) then
         Check (Driver.Pixels.Frames (High_End (T, 1).Frames) = 3, "the noisy open view lost frames");
      end if;
   end Noise_Hides_A_Small_Step;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.views.sweep", "the ends of a channel's travel are not its lowest and highest still views",
                             Ends_Of_A_Sweep'Access);
      Driver.Tests.Register ("hand.views.frames", "views too short to measure their own noise are taken as ends",
                             Single_Frames_Are_Not_Ends'Access);
      Driver.Tests.Register ("hand.views.noise", "reading noise splits one still view", Noise_Hides_A_Small_Step'Access);
   end Register;

end Driver.Robot.Hand.Views.Tests;
