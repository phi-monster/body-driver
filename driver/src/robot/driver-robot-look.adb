with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Driver.Beats;
with Driver.Commands;
with Driver.Conventions;
with Driver.Geometry;
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

   procedure At_Point (M : in out Model; A : Arm_Id; X : Vec3; Px : Driver.Images.Pixel; Reached : out Boolean) is
      E     : constant Eye_Id'Base := Kinematics.Eye_Of (M, A);
      G     : constant Group_Id := Arm_Group (M, A);
      Size  : constant Natural := Group_Size (M, G);
      Goal  : Real_Array (1 .. Size) := [others => 0.0];
      Ready : Boolean := False;
      Why   : Unbounded_String;

      procedure Solve_Now is
         O         : constant Observation := Driver.Beats.Latest.all;
         Placement : Rigid;
         Scale     : Real;
         Placed    : Boolean;
      begin
         if Natural (E) > Natural (O.Images.Length) or else not Driver.Observations.Has_Image (O, E)
           or else G > O.Readings.Last_Index or else O.Readings.Element (G)'Length /= Size
         then
            Why := To_Unbounded_String ("its eye has no picture, or the arm no readings, now");
            return;
         end if;
         Kinematics.In_World (M, A, Placement, Scale, Placed);
         if not Placed or else not (Scale > 0.0) or else not (Eye_Noise (M, E) > 0.0) then
            Why := To_Unbounded_String ("the arm is not placed in the world, or its eye's error is not measured");
            return;
         end if;
         declare
            Target : constant Vec3 := (1.0 / Scale) * (Inverse (Placement) * X);
            Q      : Real_Array := O.Readings.Element (G);
            --  The line of sight through the pixel aimed at, in the eye's frame, and the picture's rows and columns
            --  square to it.
            Mid    : constant Vec3 := Ray_In_Eye (M, E, Px.U, Px.V);
            Rows   : constant Vec3 := [1.0, 0.0, 0.0] - Mid (1) * Mid;
            Cols   : constant Vec3 := [0.0, 1.0, 0.0] - Mid (2) * Mid;
            Pixel  : constant Real :=
              Arccos (Real'Max (-1.0, Real'Min (1.0, Mid * Ray_In_Eye (M, E, Px.U + 1.0, Px.V))));
            --  Within Z of the aligner's error of that pixel, as an angle: there.
            Near   : constant Real := Driver.Conventions.Z * Eye_Noise (M, E) * Pixel;
            --  How far the line of sight to the point turns from the middle one at those readings, as an angle along
            --  the picture's rows and columns: the whole angle, up to a half turn, so a point behind the eye is
            --  turned to as one in front is (s62: the black box lay behind the arm's eye where it stood).
            function Off (Readings : Real_Array; U, V : out Real) return Boolean is
               P      : constant Vec3 := Inverse (Kinematics.Eye_In_Reference (M, A, Readings)) * Target;
               D      : constant Vec3 := (if abs P > 0.0 then Unit (P) else Mid);
               Across : constant Vec3 := D - Real'(D * Mid) * Mid;
               Angle  : constant Real := Arctan (abs Across, D * Mid);
            begin
               U := 0.0;
               V := 0.0;
               if abs Across > 0.0 then
                  U := Angle * (Unit (Across) * Unit (Rows));
                  V := Angle * (Unit (Across) * Unit (Cols));
               end if;
               return abs P > 0.0;
            end Off;
         begin
            for Round in 1 .. Real'Machine_Mantissa loop
               declare
                  U0, V0 : Real;
                  J      : array (1 .. 2, 1 .. Size) of Real := [others => [others => 0.0]];
                  AA, AB, BB : Real := 0.0;
               begin
                  if not Off (Q, U0, V0) then
                     Why := To_Unbounded_String ("the point is where its eye is");
                     return;
                  end if;
                  if Sqrt (U0 ** 2 + V0 ** 2) <= Near then
                     Ready := True;
                     exit;
                  end if;
                  --  How the point's angle from the middle moves per unit of each joint the keyframes measured, by a
                  --  turn that moves the picture a pixel.
                  for C in 1 .. Size loop
                     declare
                        Shift : constant Real := Kinematics.Pixels_Per_Unit (M, A, C);
                        Moved : Real_Array := Q;
                        U1, V1 : Real;
                     begin
                        if Shift > 0.0 then
                           Moved (C) := Moved (C) + 1.0 / Shift;
                           if Off (Moved, U1, V1) then
                              J (1, C) := (U1 - U0) * Shift;
                              J (2, C) := (V1 - V0) * Shift;
                           end if;
                        end if;
                     end;
                  end loop;
                  for C in 1 .. Size loop
                     AA := AA + J (1, C) ** 2;
                     AB := AB + J (1, C) * J (2, C);
                     BB := BB + J (2, C) ** 2;
                  end loop;
                  declare
                     Det : constant Real := AA * BB - AB ** 2;
                  begin
                     if not (Det > 0.0) then
                        Why := To_Unbounded_String ("no joint moves the point across the picture");
                        return;
                     end if;
                     declare
                        --  The least turn that takes the point to the middle, to first order: J' (J J')^-1 (-r).
                        Y1    : constant Real := (BB * (-U0) - AB * (-V0)) / Det;
                        Y2    : constant Real := (AA * (-V0) - AB * (-U0)) / Det;
                        Step  : Real_Array (1 .. Size);
                        Better : Boolean := False;
                     begin
                        for C in 1 .. Size loop
                           Step (C) := J (1, C) * Y1 + J (2, C) * Y2;
                        end loop;
                        --  Halved until it brings the point nearer the middle.
                        for Halving in 1 .. Real'Machine_Mantissa loop
                           declare
                              Next   : constant Real_Array := [for C in 1 .. Size => Q (C) + Step (C)];
                              U1, V1 : Real;
                           begin
                              if Off (Next, U1, V1) and then U1 ** 2 + V1 ** 2 < U0 ** 2 + V0 ** 2 then
                                 Q := Next;
                                 Better := True;
                                 exit;
                              end if;
                              Step := [for C in 1 .. Size => Step (C) / 2.0];
                           end;
                        end loop;
                        if not Better then
                           Why := To_Unbounded_String ("no turn brings the point nearer the middle of its picture");
                           return;
                        end if;
                     end;
                  end;
               end;
            end loop;
            Goal := Q;
         end;
      end Solve_Now;

      Report  : Driver.Robot.Motion.Step_Report;
      Targets : Driver.Commands.Command := Driver.Commands.Hold;
   begin
      Reached := False;
      if E = 0 or else not Kinematics.Fitted (M, A) or else Eye_Mount (M, E).Kind /= Arm_Carried then
         return;
      end if;
      Driver.Beats.Within_A_Beat (Solve_Now'Access);
      if not Ready then
         Driver.Log.Line (Driver.Log.Robot, "look: arm" & A'Image & " does not turn its eye to the point: "
                          & To_String (Why));
         return;
      end if;
      Driver.Commands.Set_Target (Targets, G, Goal);
      Driver.Robot.Motion.Step (M, Targets, Report, Land => True);
      Reached := Report.Outcome = Driver.Robot.Motion.Reached;
      if Reached then
         declare
            Waited : Natural;
         begin
            Driver.Robot.Motion.Settle (M, Waited);
         end;
      end if;
      Driver.Log.Line (Driver.Log.Robot, "look: arm" & A'Image & " turned its eye to the point, to its pixel ("
                       & Driver.Log.Image (Px.U, 1) & ", " & Driver.Log.Image (Px.V, 1) & "): "
                       & Report.Outcome'Image & " after" & Report.Beats'Image & " beats"
                       & (if Reached then "" else ": " & To_String (Report.Detail)));
   end At_Point;

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
                  Back    : Driver.Robot.Motion.Step_Report;
                  Reached : constant Boolean := Report.Outcome = Driver.Robot.Motion.Reached;
                  Waited  : Natural;
               begin
                  if Reached then
                     Taken := Taken + 1;
                     --  Held there until the picture rests too: the world keeps a view only at a still beat, and the
                     --  readings settle before the picture does (s64: none of the look's places was kept as a view,
                     --  every one left as its readings came to rest, its eye's picture three beats behind them).
                     Driver.Robot.Motion.Settle (M, Waited);
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
