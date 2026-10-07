with Ada.Numerics.Long_Elementary_Functions;
with Driver.Tests;

package body Driver.Robot.Hand.Aims.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   Eye   : constant Vec3 := [0.05, 0.0, 0.05];
   Lobe1 : constant Vec3 := Unit ([0.13, 0.04, 0.0] - Eye);
   Lobe2 : constant Vec3 := Unit ([0.13, -0.04, 0.0] - Eye);
   Down  : constant Vec3 := [0.0, 0.0, -1.0];

   procedure Turned_Keeps_The_Eye is
      Tool : constant Rigid := (Rotation => Exp ([0.4, -0.2, 1.1]), Translation => [0.3, -0.1, 0.4]);
      Away : constant Vec3 := Away_From (Lobe1, [1 => Lobe2]);
   begin
      for Tilt of Real_Vector'[0.0, 0.3, 0.9] loop
         declare
            Along : constant Vec3 := Tilted (Lobe1, Away, Tilt);
            Goal  : constant Rigid := Turned_About (Tool, Eye, Along, Down);
         begin
            Check (abs (Goal * Eye - Tool * Eye) < 1.0e-12, "the turn moved the eye");
            Check (abs (Goal.Rotation * Along - Down) < 1.0e-12, "the tilted line does not point down");
            --  The tilt is the angle between the line pressed along and the lobe's own.
            Check (abs (Arccos (Real'Min (1.0, Along * Lobe1)) - Tilt) < 1.0e-9, "the tilt is not its size");
            --  Tilting away from the other lobe raises it above the aimed one.
            if Tilt > 0.0 then
               Check (Down * (Goal.Rotation * Lobe2) < Down * (Goal.Rotation * Lobe1),
                      "the other lobe's line does not stay behind the aimed one's");
            end if;
         end;
      end loop;
      --  Pointing the opposite way is half a turn, not nothing.
      declare
         Up_Facing : constant Rigid := Turned_About (Tool, Eye, Transpose (Tool.Rotation) * (-Down), Down);
      begin
         Check (abs (Up_Facing.Rotation * (Transpose (Tool.Rotation) * (-Down)) - Down) < 1.0e-12,
                "a line pointing up was not turned down");
      end;
   end Turned_Keeps_The_Eye;

   procedure Hand_Scale is
   begin
      Check (abs (Spread (Lobe1, [1 => Lobe2]) - Arccos (Lobe1 * Lobe2)) < 1.0e-12,
             "the angle between the lines is not the hand's scale");
      Check (Spread (Lobe1, [1 .. 0 => Zero3]) = 0.0 and then abs Away_From (Lobe1, [1 .. 0 => Zero3]) = 0.0,
             "a lone lobe was given another to tilt away from");
      Check (Away_From (Lobe1, [1 => Lobe2]) * Lobe2 < 0.0 and then abs (Away_From (Lobe1, [1 => Lobe2]) * Lobe1) < 1.0e-12,
             "away from the other lobe is not across the line and away from it");
   end Hand_Scale;

   procedure Tilts_Follow_The_Arm is
      --  The least tilt a second press can tell from a stop that does not move
      --  with it: a tip 4.83 along its line known to 0.0324 (A16's first): the
      --  hits of a stop at the same height, straight and tilted by T, differ by
      --  the distance times T squared over 2, which is Z times the root of 2
      --  of the sigma at T = 0.239.
      Least : constant Real := Least_Tilt ((Value => 4.826, Sigma => 0.0324, Degrees_Of_Freedom => 0));
      Tilt, Bound : Real;
   begin
      Check (abs (Least - 0.2387) < 1.0e-3, "the least tilt of the tip A16 gave is" & Real'Image (Least));
      Check (Least_Tilt (Unknown) = Real'Last, "a tilt is worth something with the tip's distance unknown");
      --  The hand's own angle, 1.0, stopped on the arm: half of it, then a
      --  quarter, which is under the least: no more.
      Tilt := 1.0;
      Bound := Ada.Numerics.Pi / 2.0;
      Next_Tilt (Tilt, Stalled => True, Bound => Bound, Least => 0.3);
      Check (Tilt = 0.5 and then Bound = 1.0, "after a stop at 1.0 the next tilt is" & Real'Image (Tilt));
      Next_Tilt (Tilt, Stalled => True, Bound => Bound, Least => 0.3);
      Check (Tilt = 0.0 and then Bound = 0.5, "after a stop at 0.5, under the least a quarter, the next is" & Real'Image (Tilt));
      --  Presses that a tip rests on double, up to a right angle.
      Tilt := 0.3;
      Bound := Ada.Numerics.Pi / 2.0;
      Next_Tilt (Tilt, Stalled => False, Bound => Bound, Least => 0.2);
      Check (Tilt = 0.6, "a press the tip rests on at 0.3 was followed by" & Real'Image (Tilt));
      Next_Tilt (Tilt, Stalled => False, Bound => Bound, Least => 0.2);
      Check (Tilt = 1.2, "a press the tip rests on at 0.6 was followed by" & Real'Image (Tilt));
      Next_Tilt (Tilt, Stalled => False, Bound => Bound, Least => 0.2);
      Check (Tilt = 0.0, "a tilt past a right angle was made");
      --  A stop at 1.0 and a press the tip rests on at half of it: the double
      --  of that is the tilt that stopped, which is not made again.
      Tilt := 1.0;
      Bound := Ada.Numerics.Pi / 2.0;
      Next_Tilt (Tilt, Stalled => True, Bound => Bound, Least => 0.2);
      Next_Tilt (Tilt, Stalled => False, Bound => Bound, Least => 0.2);
      Check (Tilt = 0.0, "a tilt that stopped was made again after half of it did not:" & Real'Image (Tilt));
      --  With the tip's distance unknown nothing halves: an arm that stopped is not asked for less.
      Tilt := 1.0;
      Bound := Ada.Numerics.Pi / 2.0;
      Next_Tilt (Tilt, Stalled => True, Bound => Bound, Least => Real'Last);
      Check (Tilt = 0.0, "a stop was followed by half of the tilt with no tip to tell it from");
   end Tilts_Follow_The_Arm;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.aims.tilts", "a tilt the arm cannot make is asked again, or the tilts never end",
                             Tilts_Follow_The_Arm'Access);
      Driver.Tests.Register ("hand.aims.turn", "the turn that aims a lobe moves the eye or aims it wrong",
                             Turned_Keeps_The_Eye'Access);
      Driver.Tests.Register ("hand.aims.scale", "the tilts' direction or size is not the hand's own",
                             Hand_Scale'Access);
   end Register;

end Driver.Robot.Hand.Aims.Tests;
