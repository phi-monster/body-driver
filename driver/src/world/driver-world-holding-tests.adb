with Driver.Tests;

package body Driver.World.Holding.Tests is

   use Driver.Numerics.Arrays;
   use Driver.Tests;

   function At_Fraction (Value, Sigma : Real) return Estimate is
     (Value => Value, Sigma => Sigma, Degrees_Of_Freedom => 0);

   procedure Holds_While_Short is
      Took : constant Estimate := At_Fraction (0.6, 0.01);
   begin
      Check (Still_Held (Took, At_Fraction (0.6, 0.01)), "a closer standing where it took the thing let it go");
      Check (Still_Held (Took, At_Fraction (0.66, 0.01)), "a closer squeezing further let the thing go");
      Check (not Still_Held (Took, At_Fraction (0.999, 0.01)), "a closer closed on nothing still holds");
      Check (not Still_Held (Took, At_Fraction (1.02, 0.01)), "a closer past closed on nothing still holds");
      Check (not Still_Held (Took, At_Fraction (0.3, 0.01)), "a closer that opened still holds");
      Check (not Still_Held (Took, Unknown), "a closer whose reading is unknown holds");
      Check (not Still_Held (Unknown, At_Fraction (0.6, 0.01)), "a hold taken at an unknown reading holds");
   end Holds_While_Short;

   procedure Carried_With_The_Tool is
      Small  : constant Mat3 := 1.0e-8 * Identity3;
      From   : constant Pose_Estimate :=
        (Pose => (Rotation => Exp ([0.0, 0.0, 0.3]), Translation => [0.2, -0.1, 0.15]),
         Position_Covariance => 4.0e-6 * Identity3, Rotation_Covariance => 1.0e-6 * Identity3);
      --  Lifted 0.1 up and turned a third of a radian about a slanted axis.
      To     : constant Pose_Estimate :=
        (Pose => (Rotation => Exp ([0.1, -0.2, 0.25]) * Exp ([0.0, 0.0, 0.3]), Translation => [0.22, -0.08, 0.25]),
         Position_Covariance => 9.0e-6 * Identity3, Rotation_Covariance => 2.0e-6 * Identity3);
      Motion : constant Rigid := To.Pose * Inverse (From.Pose);
      Before : Driver.World.Pairs.Match_Vectors.Vector;
      Up     : Driver.World.Estimates.Normal_Vectors.Vector;
      Points : Driver.World.Pairs.Match_Vectors.Vector;
      Normals : Driver.World.Estimates.Normal_Vectors.Vector;
   begin
      --  A box's corners on the table below the tool, their normals up.
      for X in 0 .. 1 loop
         for Y in 0 .. 1 loop
            Before.Append (Driver.World.Pairs.Match'
                             (In_First => (U => 0.0, V => 0.0), In_Second => (U => 0.0, V => 0.0),
                              Point => (Mean => [0.18 + 0.04 * Real (X), -0.12 + 0.03 * Real (Y), 0.02],
                                        Covariance => Small),
                              First => 1));
            Up.Append (Vec3'[0.0, 0.0, 1.0]);
         end loop;
      end loop;
      Points := Before;
      Normals := Up;
      Carry (Points, Normals, From, To);
      for K in Before.First_Index .. Before.Last_Index loop
         Check (abs (Points (K).Point.Mean - Motion * Before (K).Point.Mean) < 1.0e-12,
                "a held point did not go where the tool took it");
         Check (abs (Vec3'(Normals (K) - Times (Motion.Rotation, Up (K)))) < 1.0e-12,
                "a held point's normal did not turn with the tool");
         for L in K + 1 .. Before.Last_Index loop
            Check (abs (Real'(abs (Points (K).Point.Mean - Points (L).Point.Mean))
                        - Real'(abs (Before (K).Point.Mean - Before (L).Point.Mean))) < 1.0e-12,
                   "two held points came apart or together");
         end loop;
         --  No better known than the two tool poses that carried it.
         for A in 1 .. 3 loop
            Check (Points (K).Point.Covariance (A, A)
                     >= From.Position_Covariance (A, A) + To.Position_Covariance (A, A),
                   "a held point is carried better than the tool's own poses are known");
         end loop;
      end loop;
      --  A tool that did not move leaves them where they were.
      Points := Before;
      Normals := Up;
      Carry (Points, Normals, From, From);
      for K in Before.First_Index .. Before.Last_Index loop
         Check (abs (Points (K).Point.Mean - Before (K).Point.Mean) < 1.0e-12,
                "a held point moved while the tool stood still");
      end loop;
   end Carried_With_The_Tool;

   procedure Register is
   begin
      Driver.Tests.Register ("world.holding.short", "a closer that took a thing is taken to hold it after it closed on "
                             & "nothing or opened, or to let it go while it squeezes", Holds_While_Short'Access);
      Driver.Tests.Register ("world.holding.carried", "a held thing's points do not go rigidly where the tool goes, "
                             & "or are carried better than the tool is known", Carried_With_The_Tool'Access);
   end Register;

end Driver.World.Holding.Tests;
