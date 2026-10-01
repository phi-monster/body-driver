with Ada.Numerics;
with Ada.Text_IO;
with Driver.Action.Plants.Tests;
with Driver.Action.Snapshots;
with Driver.Action.Snapshots.Tests;
with Driver.Clock;
with Driver.Log;
with Driver.Numerics;
with Driver.Tests;

package body Driver.Action.Execution.Tests is

   use Driver.Numerics;
   use Driver.Tests;
   use Driver.Action.Snapshots;
   use Driver.Action.Snapshots.Tests;
   use type Driver.Action.Snapshots.Hand_Id;

   package Sim renames Driver.Action.Plants.Tests;

   Pi : constant := Ada.Numerics.Pi;

   Sigma : constant Real := 0.0005;
   Pitch : constant Real := 0.005;

   Upright : constant Rigid := Identity;
   Turned  : constant Rigid := (Rotation => Exp ([0.9, -1.3, 2.2]), Translation => [2.0, 1.0, -0.5]);

   type Rigid_Array is array (Positive range <>) of Rigid;
   Both_Frames : constant Rigid_Array := [Upright, Turned];

   --  Tool z pointing down at the table.
   function Down_At (X, Y, Z : Real) return Rigid is ((Rotation => Exp ([Pi, 0.0, 0.0]), Translation => [X, Y, Z]));

   function On_Table (X, Y, About : Real) return Rigid is
     ((Rotation => Exp ([0.0, 0.0, About]), Translation => [X, Y, 0.0]));

   --  One arm with a two-lobe gripper above the table, late by two beats and
   --  delivering 70 to 85 percent of every step.
   procedure One_Gripper (W : in out Sim.World; Place : Rigid; Seed : Integer) is
   begin
      Sim.Start (W, Place, Sigma, Pitch, Seed);
      Sim.Add_Arm (W, Base => [0.0, -0.3, 0.0], Reach => 0.8, Tool => Down_At (0.0, -0.1, 0.25), Lag => 2,
                   Rate => 0.5, Delivery_Low => 0.7, Delivery_High => 0.85, Wrist => Pi, Tilt => Pi / 2.0);
      Sim.Add_Gripper (W, 1, Opening => 0.08, Width => 0.015, Thickness => 0.01, Depth => 0.04);
   end One_Gripper;

   function Endings (E : Ending) return Ending_Set is
      S : Ending_Set := [others => False];
   begin
      S (E) := True;
      return S;
   end Endings;

   function Height_Want (T : Thing_Id; Up : Boolean; Until_Ending : Ending; Steps : Natural := 0) return Want is
     ((Kind => Change, Until_Endings => Endings (Until_Ending), Max_Steps => Steps, Eye => Any_Eye, Anyway => False,
       Thing => T, Quantity => 1, Increase => Up));

   procedure Run (W : in out Sim.World; Wanted : Want; R : out Result) is
      Start_Beat : constant Natural := W.Beat;
      Start_Time : constant Duration := Driver.Clock.Seconds;
   begin
      Execute (W, Wanted, R);
      Ada.Text_IO.Put_Line ("      " & Ending'Image (R.Final) & " after " & Driver.Log.Image (W.Beat - Start_Beat)
                            & " beats, " & Driver.Log.Image (Real (Driver.Clock.Seconds - Start_Time), 2) & " s: "
                            & To_String (R.Account) & (if Length (R.Tried) > 0 then " | tried: " & To_String (R.Tried)
                                                       else ""));
   end Run;

   procedure Bar_Up_And_Down is
   begin
      for Place of Both_Frames loop
         declare
            W : Sim.World;
            R : Result;
         begin
            One_Gripper (W, Place, 7);
            Sim.Add_Thing (W, Bar (0.2, 0.02, 0.02), On_Table (0.1, 0.05, 0.3), Mu => 0.6);
            Run (W, Height_Want (1, True, Free), R);
            Check (R.Final = Free, "a bar lifted until free does not end free");
            Check (Sim.Lowest (W, 1) > 0.0, "a bar said to be free still lies on the table");
            Check (Sim.Truth (W, 1).Held_By = 1, "a lifted bar is not in the hand");
            Run (W, Height_Want (1, False, Touched), R);
            Check (R.Final = Touched, "a bar put down does not end touched");
            Check (abs Sim.Lowest (W, 1) < Pitch, "a bar put down is not on the table");
            Check (Sim.Truth (W, 1).Held_By = 0, "a bar put down on the table is not let go");
         end;
      end loop;
   end Bar_Up_And_Down;

   procedure Register is
   begin
      Register ("action.run.bar", "a bar is not lifted off the table and put back, or not let go",
                Bar_Up_And_Down'Access);
   end Register;

end Driver.Action.Execution.Tests;
