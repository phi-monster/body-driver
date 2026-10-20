--  A study, not the driver: can a fixed eye be placed by where an arm's closer moves in its picture? The fake world
--  of one seed is posed at arms' joints drawn about rest, its fixed eye rendered with the closer at both ends, and
--  the centre of the pixels that changed set against where the truth's camera puts points of the tool's frame. If
--  one point of the tool's frame lands on every such centre within a pixel or two, the closer's motion is an anchor a
--  fixed eye can be placed by (August's anchors, the part an appearance matcher called the hand, were not: the
--  camera stuck at 30 cm). The tool frame's best point is fitted against the true camera; its residuals say how well
--  one point explains the centres, over every pose and over the poses whose tool turned least from rest.
--
--  probe_study SEED POSES SPREAD: SPREAD the joints' draw about rest, in radians.

with Ada.Command_Line;
with Ada.Text_IO;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver;
with Driver.Numerics;
with Driver.Numerics.Dense;
with Fake_World;
with Fake_Render;

procedure Probe_Study is
   use Ada.Text_IO;
   use Ada.Numerics.Long_Elementary_Functions;
   use Driver;
   use Driver.Numerics;
   use Driver.Numerics.Arrays;
   use Fake_World;

   Seed   : constant Natural := Natural'Value (Ada.Command_Line.Argument (1));
   Poses  : constant Positive := Positive'Value (Ada.Command_Line.Argument (2));
   Spread : constant Real := Real'Value (Ada.Command_Line.Argument (3));
   W      : World;
   T      : constant Template := (Hands => [Jaw, Jaw], Dead_Arm => 2, Ticks => 4096, Servo => True, others => <>);
   Gen    : Ada.Numerics.Float_Random.Generator;
   Fixed  : Natural := 0;
   Tool   : Positive := 1;
   Closer : Natural := 0;

   type Sample is record
      Pose     : Rigid;      --  the tool in the world
      U, V     : Real;       --  the changed pixels' centre
      Count    : Natural;    --  how many changed
      Turned   : Real;       --  the tool's turn from rest, radians
   end record;
   Samples : array (1 .. Poses) of Sample;
   Kept    : Natural := 0;
   Rest_Q  : array (1 .. 64) of Real := [others => 0.0];
   Rest_Tool : Rigid;

   function Luma (P : Real_Array; I : Natural) return Real is
     ((P (P'First + 3 * I) + P (P'First + 3 * I + 1) + P (P'First + 3 * I + 2)) / 3.0);

   procedure Project (C : Camera; X : Vec3; U, V : out Real; Front : out Boolean) is
      Y : constant Vec3 := Inverse (C.Pose) * X;
   begin
      Front := Y (3) > 0.0;
      U := (if Front then C.Fx * Y (1) / Y (3) + C.Cx else 0.0);
      V := (if Front then C.Fy * Y (2) / Y (3) + C.Cy else 0.0);
   end Project;

   --  The tool frame's point whose projections fit the centres best (Gauss-Newton from the tool's origin), and the
   --  root mean square of what it leaves, over the samples Use takes.
   procedure Best_Point (Cam : Camera; Use_It : access function (S : Sample) return Boolean;
                         A : out Vec3; Rms : out Real; Used : out Natural) is
      Step : constant Real := 1.0E-6;
   begin
      A := [0.0, 0.0, 0.0];
      for Pass in 1 .. 30 loop
         declare
            JtJ : Mat3 := [others => [others => 0.0]];
            Jtr : Vec3 := [0.0, 0.0, 0.0];
            Sum : Real := 0.0;
         begin
            Used := 0;
            for K in 1 .. Kept loop
               if Use_It (Samples (K)) then
                  declare
                     U0, V0 : Real;
                     F0     : Boolean;
                     Ju, Jv : Vec3;
                  begin
                     Project (Cam, Samples (K).Pose * A, U0, V0, F0);
                     if F0 then
                        for D in 1 .. 3 loop
                           declare
                              B      : Vec3 := A;
                              U1, V1 : Real;
                              F1     : Boolean;
                           begin
                              B (D) := B (D) + Step;
                              Project (Cam, Samples (K).Pose * B, U1, V1, F1);
                              Ju (D) := (U1 - U0) / Step;
                              Jv (D) := (V1 - V0) / Step;
                           end;
                        end loop;
                        for I in 1 .. 3 loop
                           for J in 1 .. 3 loop
                              JtJ (I, J) := JtJ (I, J) + Ju (I) * Ju (J) + Jv (I) * Jv (J);
                           end loop;
                           Jtr (I) := Jtr (I) + Ju (I) * (Samples (K).U - U0) + Jv (I) * (Samples (K).V - V0);
                        end loop;
                        Sum := Sum + (Samples (K).U - U0) ** 2 + (Samples (K).V - V0) ** 2;
                        Used := Used + 1;
                     end if;
                  end;
               end if;
            end loop;
            Rms := (if Used > 0 then Sqrt (Sum / Real (Used)) else 0.0);
            exit when Used < 2;
            declare
               Lower    : Mat3;
               Definite : Boolean;
            begin
               Driver.Numerics.Dense.Cholesky (JtJ, Lower, Definite);
               exit when not Definite;
               declare
                  Delta_A : constant Vec3 := Driver.Numerics.Dense.Cholesky_Solve (Lower, Jtr);
               begin
                  A := A + Delta_A;
                  exit when abs Delta_A < 1.0E-9;
               end;
            end;
         end;
      end loop;
   end Best_Point;

   function Every (S : Sample) return Boolean is (S.Count > 0);
   Least_Turn : Real := Real'Last;
   function Little (S : Sample) return Boolean is (S.Count > 0 and then S.Turned <= Least_Turn);
begin
   Ada.Numerics.Float_Random.Reset (Gen, Seed);
   Build (W, Seed, 320, 240, T);
   Forward (W);
   for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
      if W.Cameras (C).Link = 0 then
         Fixed := C;
      end if;
   end loop;
   Tool := W.Tools (1);
   Closer := W.Closers (1);
   for L in W.Links.First_Index .. W.Links.Last_Index loop
      Rest_Q (L) := W.Links (L).Q;
   end loop;
   Rest_Tool := W.Links (Tool).Pose;
   Put_Line ("seed" & Seed'Image & ": fixed camera" & Fixed'Image & ", tool link" & Tool'Image & ", closer link"
             & Closer'Image & " (" & W.Links (Closer).Lo'Image & " .." & W.Links (Closer).Hi'Image & ")");
   declare
      Cam : aliased Camera := W.Cameras (Fixed);
      N   : constant Natural := Cam.Width * Cam.Height;
      type Pixel_Access is access Real_Array;
      A    : constant Pixel_Access := new Real_Array (1 .. 3 * N);
      B    : constant Pixel_Access := new Real_Array (1 .. 3 * N);
   begin
      for K in 1 .. Poses loop
         for L in W.Links.First_Index .. W.Links.Last_Index loop
            if W.Links (L).Arm = 1 and then W.Links (L).Kind = Revolute and then L /= Closer then
               declare
                  Draw : constant Real := Spread * (2.0 * Real (Ada.Numerics.Float_Random.Random (Gen)) - 1.0);
               begin
                  W.Links (L).Q := Real'Max (W.Links (L).Lo, Real'Min (W.Links (L).Hi, Rest_Q (L) + Draw / W.Links (L).Unit));
               end;
            end if;
         end loop;
         W.Links (Closer).Q := W.Links (Closer).Lo;
         Forward (W);
         Cam := W.Cameras (Fixed);
         Fake_Render.Render (W, Cam, A.all);
         W.Links (Closer).Q := W.Links (Closer).Hi;
         Forward (W);
         Cam := W.Cameras (Fixed);
         Fake_Render.Render (W, Cam, B.all);
         declare
            Su, Sv : Real := 0.0;
            Count  : Natural := 0;
         begin
            for I in 0 .. N - 1 loop
               if abs (Luma (A.all, I) - Luma (B.all, I)) > 1.0 then
                  Su := Su + Real (I mod Cam.Width) + 0.5;
                  Sv := Sv + Real (I / Cam.Width) + 0.5;
                  Count := Count + 1;
               end if;
            end loop;
            if Count > 0 then
               Kept := Kept + 1;
               Samples (Kept) := (Pose   => W.Links (Tool).Pose,
                                  U      => Su / Real (Count),
                                  V      => Sv / Real (Count),
                                  Count  => Count,
                                  Turned => Angle (Transposed (Rest_Tool.Rotation) * W.Links (Tool).Pose.Rotation));
            end if;
         end;
      end loop;
      Put_Line ("poses" & Poses'Image & ", the closer seen moving at" & Kept'Image);
      declare
         Turns : array (1 .. Kept) of Real;
      begin
         for K in 1 .. Kept loop
            Turns (K) := Samples (K).Turned;
         end loop;
         --  The half of the poses whose tool turned least from rest.
         for I in 1 .. Kept loop
            for J in I + 1 .. Kept loop
               if Turns (J) < Turns (I) then
                  declare
                     X : constant Real := Turns (I);
                  begin
                     Turns (I) := Turns (J);
                     Turns (J) := X;
                  end;
               end if;
            end loop;
         end loop;
         if Kept > 0 then
            Least_Turn := Turns (Natural'Max (1, Kept / 2));
         end if;
      end;
      declare
         A1, A2 : Vec3;
         R1, R2 : Real;
         U1, U2 : Natural;
         Pixels_A_Metre : constant Real := Cam.Fx / abs (Rest_Tool.Translation - Cam.Pose.Translation);
      begin
         Best_Point (Cam, Every'Access, A1, R1, U1);
         Best_Point (Cam, Little'Access, A2, R2, U2);
         Put_Line ("every pose:" & U1'Image & " samples, best tool point" & A1 (1)'Image & A1 (2)'Image & A1 (3)'Image
                   & " m, rms" & R1'Image & " px (about" & Real'Image (R1 / Pixels_A_Metre * 1000.0) & " mm at the arm)");
         Put_Line ("tool turned at most" & Least_Turn'Image & " rad:" & U2'Image & " samples, best tool point"
                   & A2 (1)'Image & A2 (2)'Image & A2 (3)'Image & " m, rms" & R2'Image & " px (about"
                   & Real'Image (R2 / Pixels_A_Metre * 1000.0) & " mm)");
         for K in 1 .. Natural'Min (Kept, 8) loop
            Put_Line ("  sample" & K'Image & ": changed" & Samples (K).Count'Image & " px at" & Samples (K).U'Image
                      & Samples (K).V'Image & ", tool turned" & Samples (K).Turned'Image);
         end loop;
      end;
   end;
end Probe_Study;
