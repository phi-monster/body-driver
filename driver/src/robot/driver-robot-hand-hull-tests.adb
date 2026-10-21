with Ada.Numerics;
with Ada.Streams;
with Ada.Numerics.Long_Elementary_Functions;
with Ada.Strings.Unbounded;
with Driver.Bytes;
with Driver.Images;
with Driver.Tests;

package body Driver.Robot.Hand.Hull.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   --  The eye: a pinhole of focal length 140 pixels over 320 by 240, as the fake robot's fixed eye is.
   Width  : constant := 320;
   Height : constant := 240;
   Focal  : constant := 140.0;
   Cu     : constant := 160.0;
   Cv     : constant := 120.0;

   procedure Pinhole (P : Vec3; U, V : out Real; In_Front : out Boolean) is
   begin
      In_Front := P (3) > 0.0;
      if In_Front then
         U := Cu + Focal * P (1) / P (3);
         V := Cv + Focal * P (2) / P (3);
      else
         U := 0.0;
         V := 0.0;
      end if;
   end Pinhole;

   --  The hand in its tool frame, its eye at the origin and its axis along z: a palm from z 1 to 2, and two fingers
   --  from it to their tips at z 5, 0.8 thick across the way they close (x) and 1.2 wide (y), 1.4 apart: a thing goes
   --  3 into it between the tips, from their middle (0, 0, 5) toward the eye.
   type Box is record
      Low, High : Vec3;
   end record;
   Boxes : constant array (1 .. 3) of Box :=
     [(Low => [-1.5, -1.0, 1.0], High => [1.5, 1.0, 2.0]),
      (Low => [-1.5, -0.6, 2.0], High => [-0.7, 0.6, 5.0]),
      (Low => [0.7, -0.6, 2.0], High => [1.5, 0.6, 5.0])];
   Depth : constant := 3.0;

   --  Whether the ray from Origin along Way (tool frame) meets a box of the hand.
   function Meets (Origin, Way : Vec3) return Boolean is
   begin
      for B of Boxes loop
         declare
            Near : Real := 0.0;
            Far  : Real := Real'Last;
            Ok   : Boolean := True;
         begin
            for I in 1 .. 3 loop
               if Way (I) = 0.0 then
                  if Origin (I) < B.Low (I) or else Origin (I) > B.High (I) then
                     Ok := False;
                  end if;
               else
                  declare
                     T1 : constant Real := (B.Low (I) - Origin (I)) / Way (I);
                     T2 : constant Real := (B.High (I) - Origin (I)) / Way (I);
                  begin
                     Near := Real'Max (Near, Real'Min (T1, T2));
                     Far := Real'Min (Far, Real'Max (T1, T2));
                  end;
               end if;
            end loop;
            if Ok and then Near <= Far then
               return True;
            end if;
         end;
      end loop;
      return False;
   end Meets;

   --  A table behind the hand with a texture of its own, and a noise of about two levels drawn from the pixel's place.
   function Backdrop (C, R : Natural) return Real is
     (120.0 + 25.0 * Sin (0.21 * Real (C)) * Cos (0.17 * Real (R)) + 10.0 * Sin (0.05 * Real (C) + 0.07 * Real (R)));

   function Noise (C, R, K : Natural) return Real is
     (2.0 * Sin (Real (C * 7919 + R * 104729 + K * 1299709)));

   --  The pose of the tool in the eye: its axis turned up the picture (tool z to eye -y, tool y away from the eye),
   --  rolled about its axis by Roll, with the middle of the hand Across to the side of the picture's centre at 25.
   function Pose_Of (Roll, Across : Real) return Rigid is
      Turn_Up : constant Mat3 := [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]];
      Rolled  : constant Mat3 := [[Cos (Roll), -Sin (Roll), 0.0], [Sin (Roll), Cos (Roll), 0.0], [0.0, 0.0, 1.0]];
      R       : constant Mat3 := Turn_Up * Rolled;
   begin
      return (Rotation => R, Translation => [Across, 0.0, 25.0] - R * [0.0, 0.0, 3.0]);
   end Pose_Of;

   function Picture_Of (Pose : Rigid; K : Natural) return Driver.Images.Image is
      RGB   : Driver.Bytes.Byte_Array (1 .. Ada.Streams.Stream_Element_Offset (3 * Width * Height));
      Back  : constant Rigid := Inverse (Pose);
   begin
      for R in 0 .. Height - 1 loop
         for C in 0 .. Width - 1 loop
            declare
               Ray   : constant Vec3 := [(Real (C) + 0.5 - Cu) / Focal, (Real (R) + 0.5 - Cv) / Focal, 1.0];
               Value : Real :=
                 (if Meets (Back.Translation, Back.Rotation * Ray) then 40.0 else Backdrop (C, R)) + Noise (C, R, K);
               Byte  : Driver.Bytes.Byte;
            begin
               Value := Real'Max (0.0, Real'Min (255.0, Value));
               Byte := Driver.Bytes.Byte (Natural (Value));
               RGB (Ada.Streams.Stream_Element_Offset (3 * (R * Width + C) + 1)) := Byte;
               RGB (Ada.Streams.Stream_Element_Offset (3 * (R * Width + C) + 2)) := Byte;
               RGB (Ada.Streams.Stream_Element_Offset (3 * (R * Width + C) + 3)) := Byte;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Width, Height, RGB);
   end Picture_Of;

   --  The hand at places across the picture, rolled at each to Rolls; where it is in each picture, within the
   --  reach of its box's farthest corner from its middle (0, 0, 3), 2.7, at 25 from the eye, and the blur.
   function Views_Of (Rolls : Real_Array; Places : Real_Array := [-8.0, 0.0, 8.0]) return View_Array is
      Result : View_Array (1 .. Places'Length * Rolls'Length);
      N      : Natural := 0;
   begin
      for P of Places loop
         for Roll of Rolls loop
            N := N + 1;
            declare
               Pose   : constant Rigid := Pose_Of (Roll, P);
               Middle : constant Vec3 := Pose * [0.0, 0.0, 3.0];
               U, V   : Real;
               Front  : Boolean;
            begin
               Pinhole (Middle, U, V, Front);
               Result (N) := (Picture => Picture_Of (Pose, N), Eye_From_Tool => Pose, Blur => 0.5,
                              Hand_At => (U => U, V => V), Hand_Radius => Focal * 2.7 / 25.0 + 0.5);
            end;
         end loop;
      end loop;
      return Result;
   end Views_Of;

   --  A step of half the pixel's footprint at the hand.
   Step : constant Real := 25.0 / Focal / 2.0;

   procedure Seen_Between is
      Q     : constant Real := Ada.Numerics.Pi / 4.0;
      Found : constant Run := Free_Run (Views_Of ([0.0, Q, 2.0 * Q, 3.0 * Q]), Pinhole'Access, [0.0, 0.0, 5.0],
                                        [0.0, 0.0, 0.0], Step);
   begin
      Check (Known (Found.Length), "a hand seen across its fingers has no depth: "
             & Ada.Strings.Unbounded.To_String (Found.Why));
      if Known (Found.Length) then
         --  Never deeper than it is, and short of it by no more than the pixels a point may land on reach past it (the
         --  blur, half a step's footprint and a pixel, at the hand's 25 from the eye) and a step.
         Check (Found.Length.Value <= Depth + Found.Length.Sigma
                and then Found.Length.Value >= Depth - (0.5 + 0.25 + 1.0) * 25.0 / Focal - Step,
                "a hand 3 deep between its fingers is" & Real'Image (Found.Length.Value) & " +-"
                & Real'Image (Found.Length.Sigma) & " deep: " & Ada.Strings.Unbounded.To_String (Found.Why));
      end if;
   end Seen_Between;

   procedure Never_Seen_Between is
      --  Rolled a quarter turn at every place, the eye looks along the way the fingers close: one hides the room
      --  between them, and nothing between the tips is seen past.
      Found : constant Run := Free_Run (Views_Of ([Ada.Numerics.Pi / 2.0]), Pinhole'Access, [0.0, 0.0, 5.0],
                                        [0.0, 0.0, 0.0], Step);
   begin
      Check (not Known (Found.Length), "a hand never seen across its fingers is" & Real'Image (Found.Length.Value)
             & " deep");
   end Never_Seen_Between;

   procedure One_Place is
      --  Rolled at one place only, the hand covers the same pixels in every picture: nothing there has a backdrop,
      --  and nothing between the tips is seen past, however the pictures look across the fingers.
      Q     : constant Real := Ada.Numerics.Pi / 4.0;
      Found : constant Run := Free_Run (Views_Of ([0.0, Q, 2.0 * Q, 3.0 * Q], Places => [0.0]), Pinhole'Access,
                                        [0.0, 0.0, 5.0], [0.0, 0.0, 0.0], Step);
   begin
      Check (not Known (Found.Length), "a hand shown at one place only, with nothing behind it seen, is"
             & Real'Image (Found.Length.Value) & " deep");
   end One_Place;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.hull.one_place", "a hand shown at one place only is given a depth from what it "
                             & "covers in every picture", One_Place'Access);
      Driver.Tests.Register ("hand.hull.depth", "a hand shown to an eye off it is not as deep between its fingers as "
                             & "it is, or deeper", Seen_Between'Access);
      Driver.Tests.Register ("hand.hull.hidden", "a hand never seen between its fingers is given a depth",
                             Never_Seen_Between'Access);
   end Register;

end Driver.Robot.Hand.Hull.Tests;
