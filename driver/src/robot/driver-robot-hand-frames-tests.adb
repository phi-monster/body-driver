with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Tests;

package body Driver.Robot.Hand.Frames.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      --  The generator returns [0, 1] with 1 included: U1 is drawn on (0, 1].
      U1 : Real;
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      loop
         U1 := Real (Ada.Numerics.Float_Random.Random (Gen));
         exit when U1 > 0.0;
      end loop;
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   function Draw (Sigmas : Vec3) return Vec3 is
     ([Sigmas (1) * Gaussian, Sigmas (2) * Gaussian, Sigmas (3) * Gaussian]);

   function Diagonal (Sigmas : Vec3) return Mat3 is
     ([[Sigmas (1) ** 2, 0.0, 0.0], [0.0, Sigmas (2) ** 2, 0.0], [0.0, 0.0, Sigmas (3) ** 2]]);

   procedure Sampled_Uncertainty is
      --  An eye half a unit off the frame's origin, turned; a line whose origin
      --  is well off the eye's centre, so the turn's lever counts too.
      Pose_Sigma  : constant Vec3 := [0.002, 0.004, 0.001];
      Turn_Sigma  : constant Vec3 := [0.01, 0.003, 0.006];
      Point_Sigma : constant Vec3 := [0.001, 0.001, 0.002];
      Line_Sigma  : constant Real := 0.004;
      Frame : constant Pose_Estimate :=
        (Pose                => (Rotation => Exp ([0.3, -0.5, 0.2]), Translation => [0.5, 0.1, -0.2]),
         Position_Covariance => Diagonal (Pose_Sigma),
         Rotation_Covariance => Diagonal (Turn_Sigma));
      Line : constant Ray_Estimate :=
        (Origin    => (Mean => [0.3, -0.2, 0.25], Covariance => Diagonal (Point_Sigma)),
         Direction => (Unit_Vector => Unit ([0.2, -0.1, 1.0]), Sigma => Line_Sigma));
      Predicted : constant Ray_Estimate := Into (Frame, Line);
      Samples   : constant := 40_000;
      Sum       : Mat3 := [others => [others => 0.0]];
      Spread    : Real := 0.0;
      U         : constant Vec3 := Predicted.Direction.Unit_Vector;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 11);
      for K in 1 .. Samples loop
         declare
            Turned : constant Mat3 := Exp (Draw (Turn_Sigma)) * Frame.Pose.Rotation;
            Moved  : constant Vec3 := Frame.Pose.Translation + Draw (Pose_Sigma);
            --  The line's own turn: a small rotation across it.
            Across : constant Vec3 := Draw ([Line_Sigma, Line_Sigma, Line_Sigma]);
            Along  : constant Vec3 := Line.Direction.Unit_Vector;
            Part   : constant Real := Across * Along;
            Bent   : constant Vec3 := Exp (Across - Part * Along) * Along;
            Origin : constant Vec3 := Turned * (Line.Origin.Mean + Draw (Point_Sigma)) + Moved;
            Seen   : constant Vec3 := Turned * Bent;
            D      : constant Vec3 := Origin - Predicted.Origin.Mean;
         begin
            Sum := Sum + Outer (D, D);
            --  The angle away from the predicted direction, squared: two
            --  components across it, each with the direction's sigma.
            Spread := Spread + Cross (Seen, U) * Cross (Seen, U);
         end;
      end loop;
      declare
         Measured : constant Mat3 := (1.0 / Real (Samples)) * Sum;
         --  A sample variance of n Gaussian draws is off by sqrt (2 / n) of itself.
         Tolerance : constant Real := Driver.Conventions.Z * Sqrt (2.0 / Real (Samples));
      begin
         for I in 1 .. 3 loop
            Check (abs (Measured (I, I) / Predicted.Origin.Covariance (I, I) - 1.0) <= Tolerance,
                   "origin variance" & I'Image & " sampled" & Real'Image (Measured (I, I)) & " predicted"
                   & Real'Image (Predicted.Origin.Covariance (I, I)));
         end loop;
         Check (abs (Sqrt (Spread / (2.0 * Real (Samples))) / Predicted.Direction.Sigma - 1.0) <= Tolerance,
                "direction sigma sampled" & Real'Image (Sqrt (Spread / (2.0 * Real (Samples))))
                & " predicted" & Real'Image (Predicted.Direction.Sigma));
      end;
   end Sampled_Uncertainty;

   procedure Unknown_Stays_Unknown is
      Line  : constant Ray_Estimate :=
        (Origin => (Mean => [0.0, 0.0, 0.0], Covariance => [others => [others => 0.0]]),
         Direction => (Unit_Vector => [0.0, 0.0, 1.0], Sigma => 0.0));
      Frame : Pose_Estimate;   --  never measured
   begin
      Check (Into (Frame, Line).Direction.Sigma = Real'Last and then not Known (Into (Frame, Line).Origin),
             "an unmeasured frame gave a known line");
   end Unknown_Stays_Unknown;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.frames.sampled", "a line taken into the tool frame carries the wrong uncertainty",
                             Sampled_Uncertainty'Access);
      Driver.Tests.Register ("hand.frames.unknown", "an unmeasured eye mount gives a known line of sight",
                             Unknown_Stays_Unknown'Access);
   end Register;

end Driver.Robot.Hand.Frames.Tests;
