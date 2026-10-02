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

   procedure Register is
   begin
      Driver.Tests.Register ("hand.aims.turn", "the turn that aims a lobe moves the eye or aims it wrong",
                             Turned_Keeps_The_Eye'Access);
      Driver.Tests.Register ("hand.aims.scale", "the tilts' direction or size is not the hand's own",
                             Hand_Scale'Access);
   end Register;

end Driver.Robot.Hand.Aims.Tests;
