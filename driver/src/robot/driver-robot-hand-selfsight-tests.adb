with Driver.Bytes;
with Driver.Tests;

package body Driver.Robot.Hand.Selfsight.Tests is

   use Driver.Tests;
   use type Driver.Bytes.Offset;

   W : constant := 64;
   H : constant := 48;

   --  The eye of an arm: a finger, a block at the left of the lower half,
   --  fixed in its picture at one grey, and a textured world that slides
   --  across the picture as the arm moves.
   function Frame (Pose : Natural) return Driver.Images.Image is
      Data : Driver.Bytes.Byte_Array (1 .. 3 * W * H);
   begin
      for Row in 0 .. H - 1 loop
         for Column in 0 .. W - 1 loop
            declare
               C : constant Integer := Column + 5 * Pose;
               R : constant Integer := Row + 3 * Pose;
               World  : constant Natural := 60 + (C * C * 7 + R * R * 13 + C * R * 5) mod 140;
               Finger : constant Boolean := Column < 16 and then Row >= H / 2;
               L : constant Driver.Bytes.Byte := Driver.Bytes.Byte (if Finger then 20 else World);
               K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Row * W + Column));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (W, H, Data);
   end Frame;

   function Rest_Moved (Before, After : Real_Array) return Boolean is (Before /= After);
   --  The rest of a body whose readings repeat exactly moved when they changed.

   procedure See (M : in out Memory; Key : Real; Pose : Natural; Still : Boolean := True; Frames : Positive := 1) is
   begin
      for I in 1 .. Frames loop
         Observe (M, [1 => Key], [1 => Real (Pose)], Still, Frame (Pose), Rest_Moved'Access);
      end loop;
   end See;

   procedure Robot_Stays_Put is
      M : Memory := Start (W, H, Key_Noise => [1 => 0.0]);
   begin
      for P in 0 .. 5 loop
         See (M, 1.0, P, Frames => 3);   --  three still frames of one pose are one pose
      end loop;
      See (M, 1.0, 6, Still => False);   --  a frame of a picture still moving is none
      Check (Poses (M, [1 => 1.0]) = 6, "poses counted" & Natural'Image (Poses (M, [1 => 1.0])) & ", not 6");
      Check (Anchored (M, [1 => 1.0]) and then not Anchored (M, [1 => 0.5]), "anchored keys are not the seen ones");
      if Anchored (M, [1 => 1.0]) then
         declare
            V : constant Driver.Pixels.View := Anchor_For (M, [1 => 1.0]);
         begin
            Check (Driver.Pixels.Frames (V) = 6, "the anchor holds" & Natural'Image (Driver.Pixels.Frames (V)) & " frames");
            Check (Driver.Pixels.Variance (V, 5, H - 5) <= 1.0 / 12.0 + 1.0e-9,
                   "a pixel of the finger varied over the poses:" & Real'Image (Driver.Pixels.Variance (V, 5, H - 5)));
            Check (Driver.Pixels.Variance (V, 40, 10) > 100.0,
                   "a pixel of the world held still over the poses:" & Real'Image (Driver.Pixels.Variance (V, 40, 10)));
         end;
      end if;
   end Robot_Stays_Put;

   procedure Settings_Are_Kept_Or_Forgotten is
      M : Memory := Start (W, H, Key_Noise => [1 => 0.0]);
   begin
      See (M, 1.0, 0);
      See (M, 1.0, 1);
      Check (Anchored (M, [1 => 1.0]), "a setting seen from two poses was not anchored");
      --  The closer passes through settings it stays at for one pose: they are
      --  forgotten when it moves on, the anchored one is not.
      See (M, 0.9, 2);
      See (M, 0.8, 3);
      Check (Poses (M, [1 => 0.9]) = 0, "a setting left after one pose was kept");
      Check (Poses (M, [1 => 0.8]) = 1, "the setting the closer is at was not seen");
      Check (Anchored (M, [1 => 1.0]), "an anchored setting was forgotten when the closer moved on");
      --  Back at the first setting, from a new pose.
      See (M, 1.0, 4);
      Check (Poses (M, [1 => 1.0]) = 3, "the anchored setting did not go on counting poses:"
             & Natural'Image (Poses (M, [1 => 1.0])));
   end Settings_Are_Kept_Or_Forgotten;

   procedure Memory_Is_Bounded is
      M : Memory := Start (W, H, Key_Noise => [1 => 0.0]);
   begin
      --  One setting seen from five poses, then more settings than memory
      --  holds, each from two.
      for P in 0 .. 4 loop
         See (M, 1.0, P);
      end loop;
      for K in 1 .. Capacity + 3 loop
         See (M, 1.0 + Real (K), 0);
         See (M, 1.0 + Real (K), 1);
      end loop;
      declare
         Anchored_Keys : Natural := Boolean'Pos (Anchored (M, [1 => 1.0]));
      begin
         for K in 1 .. Capacity + 3 loop
            Anchored_Keys := Anchored_Keys + Boolean'Pos (Anchored (M, [1 => 1.0 + Real (K)]));
         end loop;
         Check (Anchored_Keys = Capacity, "memory holds" & Natural'Image (Anchored_Keys) & " anchored settings, not"
                & Natural'Image (Capacity));
         Check (Anchored (M, [1 => 1.0]), "the setting of most poses was the one forgotten");
         Check (not Anchored (M, [1 => 2.0]), "the setting longest unseen of the fewest poses was kept");
      end;
   end Memory_Is_Bounded;

   procedure Noise_Joins_Readings is
      --  A reading with noise 0.1: 1.02 and 0.98 are one setting, 0.5 is another.
      M : Memory := Start (W, H, Key_Noise => [1 => 0.1]);
   begin
      See (M, 1.00, 0);
      See (M, 1.02, 1);
      See (M, 0.98, 2);
      Check (Poses (M, [1 => 1.0]) = 3, "readings within their noise were taken for different settings:"
             & Natural'Image (Poses (M, [1 => 1.0])));
      See (M, 0.50, 3);
      Check (Poses (M, [1 => 1.0]) = 3 and then Poses (M, [1 => 0.5]) = 1, "a reading far from the others joined them");
   end Noise_Joins_Readings;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.selfsight.poses", "what stays put in an eye's picture as the arm moves is not told "
                             & "from the world", Robot_Stays_Put'Access);
      Driver.Tests.Register ("hand.selfsight.settings", "a setting of the closer seen from many poses is forgotten, or "
                             & "one seen from one is kept", Settings_Are_Kept_Or_Forgotten'Access);
      Driver.Tests.Register ("hand.selfsight.bounded", "memory of an eye's settings grows without bound, or forgets "
                             & "the best seen", Memory_Is_Bounded'Access);
      Driver.Tests.Register ("hand.selfsight.noise", "readings within their noise are taken for different settings",
                             Noise_Joins_Readings'Access);
   end Register;

end Driver.Robot.Hand.Selfsight.Tests;
