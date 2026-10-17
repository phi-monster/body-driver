with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Driver.Beats;
with Driver.Commands;
with Driver.Conventions;
with Driver.Geometry;
with Driver.Images;
with Driver.Log;
with Driver.Robot.Kinematics;
with Driver.Robot.Motion;

package body Driver.Robot.Look is

   use Ada.Numerics.Long_Elementary_Functions;
   use Ada.Strings.Unbounded;
   use Driver.Numerics.Arrays;
   use type Driver.Robot.Motion.Step_Outcome;
   use type Driver.Observations.Camera_Id;
   use type Driver.Observations.Group_Id;

   procedure Around (M : in out Model; A : Arm_Id; Taken : out Natural) is
      E       : constant Eye_Id'Base := Kinematics.Eye_Of (M, A);
      G       : constant Group_Id := Arm_Group (M, A);
      Size    : constant Natural := Group_Size (M, G);
      Start   : Real_Array (1 .. Size) := [others => 0.0];   --  the arm's readings where it stood
      Joint   : Natural := 0;     --  the joint turned
      Turn    : Real := 0.0;      --  by how much each way, in its reading units
      Least   : Real := 0.0;      --  the least turn whose motion of the picture the aligner tells
      Gain    : Real := 0.0;      --  the parallax it gains a pixel of the picture's motion
      Ready   : Boolean := False;
      Why     : Unbounded_String;

      procedure Measure_Now is
         O : constant Observation := Driver.Beats.Latest.all;
      begin
         if Natural (E) > Natural (O.Images.Length) or else not Driver.Observations.Has_Image (O, E)
           or else G > O.Readings.Last_Index or else O.Readings.Element (G)'Length /= Size
         then
            Why := To_Unbounded_String ("its eye has no picture, or the arm no readings, now");
            return;
         end if;
         Start := O.Readings.Element (G);
         declare
            Table : constant Driver.Geometry.Plane_Estimate := Kinematics.Table (M, A);
            W     : constant Real := Real (Driver.Images.Width (O.Images (E)));
            H     : constant Real := Real (Driver.Images.Height (O.Images (E)));
            Mid   : constant Vec3 := Ray_In_Eye (M, E, W / 2.0, H / 2.0);
            Pixel : constant Real :=
              Arccos (Real'Max (-1.0, Real'Min (1.0, Mid * Ray_In_Eye (M, E, W / 2.0 + 1.0, H / 2.0))));
            Eye   : constant Rigid := Kinematics.Eye_In_Reference (M, A, Start);
            Sight : constant Vec3 := Eye.Rotation * Mid;
            Facing : constant Real := (if Driver.Geometry.Known (Table) then Table.Normal * Sight else 0.0);
            Along : constant Real :=
              (if Facing /= 0.0 then (Table.Normal * (Table.Centre - Eye.Translation)) / Facing else 0.0);
         begin
            if abs Mid = 0.0 or else not (Pixel > 0.0) or else Along <= 0.0 then
               Why := To_Unbounded_String ("its eye's middle line of sight does not meet its table in front");
               return;
            end if;
            if not (Eye_Noise (M, E) > 0.0) then
               Why := To_Unbounded_String ("how far the aligner errs on its eye's pictures is not measured");
               return;
            end if;
            for C in 1 .. Size loop
               declare
                  Shift : constant Real := Kinematics.Pixels_Per_Unit (M, A, C);
               begin
                  if Shift > 0.0 then
                     declare
                        --  The turn that moves the picture a pixel, and how far it carries the eye across its middle
                        --  line of sight, over the depth there.
                        Moved  : Real_Array := Start;
                        Step   : Vec3;
                        Across : Vec3;
                     begin
                        Moved (C) := Moved (C) + 1.0 / Shift;
                        Step := Kinematics.Eye_In_Reference (M, A, Moved).Translation - Eye.Translation;
                        Across := Step - Real'(Step * Sight) * Sight;
                        if abs Across / Along / Pixel > Gain then
                           Gain := abs Across / Along / Pixel;
                           Joint := C;
                           Turn := (W / 4.0) / Shift;
                           Least := Driver.Conventions.Z * Eye_Noise (M, E) / Shift;
                        end if;
                     end;
                  end if;
               end;
            end loop;
            if Joint = 0 then
               Why := To_Unbounded_String ("no joint is measured to move its eye's picture");
               return;
            end if;
            Ready := True;
         end;
      end Measure_Now;

      procedure Go (Readings : Real_Array; Report : out Driver.Robot.Motion.Step_Report) is
         Targets : Driver.Commands.Command := Driver.Commands.Hold;
      begin
         Driver.Commands.Set_Target (Targets, G, Readings);
         Driver.Robot.Motion.Step (M, Targets, Report, Land => True);
      end Go;
   begin
      Taken := 0;
      if E = 0 or else not Kinematics.Fitted (M, A) or else Eye_Mount (M, E).Kind /= Arm_Carried then
         return;
      end if;
      Driver.Beats.Within_A_Beat (Measure_Now'Access);
      if not Ready then
         Driver.Log.Line (Driver.Log.Robot, "look: arm" & A'Image & " does not look around: " & To_String (Why));
         return;
      end if;
      Driver.Log.Line (Driver.Log.Robot, "look: arm" & A'Image & " turns channel" & Joint'Image & " by "
                       & Driver.Log.Image (Turn, 4) & " each way, its eye's parallax at the table "
                       & Driver.Log.Image (Gain, 4) & " of its picture's motion");
      for Sign in Boolean loop
         declare
            Amount : Real := Turn;
            Report : Driver.Robot.Motion.Step_Report;
         begin
            while Amount >= Least loop
               declare
                  To : Real_Array := Start;
               begin
                  To (Joint) := To (Joint) + (if Sign then Amount else -Amount);
                  Go (To, Report);
               end;
               declare
                  Back : Driver.Robot.Motion.Step_Report;
                  Reached : constant Boolean := Report.Outcome = Driver.Robot.Motion.Reached;
               begin
                  if Reached then
                     Taken := Taken + 1;
                  end if;
                  Driver.Log.Line (Driver.Log.Robot, "look: arm" & A'Image & " channel" & Joint'Image & " "
                                   & (if Sign then "+" else "-") & Driver.Log.Image (Amount, 4) & ": "
                                   & Report.Outcome'Image & " after" & Report.Beats'Image & " beats"
                                   & (if Reached then "" else ": " & To_String (Report.Detail)));
                  Go (Start, Back);
                  exit when Reached;
               end;
               Amount := Amount / 2.0;
            end loop;
         end;
      end loop;
      Driver.Log.Line (Driver.Log.Robot, "look: arm" & A'Image & " held its eye at" & Taken'Image
                       & " places apart and brought it back");
   end Around;

end Driver.Robot.Look;
