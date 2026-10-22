--  The fake robot's pictures beat by beat, with no driver: the body of a seed built as fake_robot builds it
--  (Fake_World.Build), every commandable group asked every Every beats to a place drawn between its joints' ends, and
--  each camera's picture made as fake_robot makes it (Fake_Render.Render, the settle's accumulation,
--  Fake_Camera.Expose with the sensor's noise). Each beat a line per camera: the hash of its render and of the
--  picture it sends. Two builds that print the same lines make the same pictures to the bit: for changes to the
--  renderer or the camera that must change only how fast they are. The last line: how long a picture took to render
--  and to expose.
--
--    fake_frames SEED BEATS [--class humanoid|base|drone] [--servo yes] [--every N]

with Ada.Calendar;
with Ada.Command_Line;
with Ada.Environment_Variables;
with Ada.Streams.Stream_IO;
with Ada.Strings.Fixed;
with Ada.Numerics.Float_Random;
with Ada.Text_IO;
with Ada.Unchecked_Conversion;
with Interfaces;
with Driver;
with Driver.Bytes;
with Fake_Camera;
with Fake_GPU;
with Fake_Render;
with Fake_World;

procedure Fake_Frames is
   use Ada.Text_IO;
   use Interfaces;
   use Driver;
   use Fake_World;
   use type Ada.Calendar.Time;
   use type Driver.Bytes.Offset;

   function Option (Name : String; Default : String := "") return String;
   function Option (Name : String; Default : String := "") return String is
   begin
      for I in 1 .. Ada.Command_Line.Argument_Count - 1 loop
         if Ada.Command_Line.Argument (I) = Name then
            return Ada.Command_Line.Argument (I + 1);
         end if;
      end loop;
      return Default;
   end Option;

   Seed   : constant Natural := Natural'Value (Ada.Command_Line.Argument (1));
   Beats  : constant Positive := Positive'Value (Ada.Command_Line.Argument (2));
   Every  : constant Positive := Positive'Value (Option ("--every", "15"));
   Width  : constant Positive := 320;
   Height : constant Positive := 240;
   W      : World;
   Shape  : Template;
   Draws  : Ada.Numerics.Float_Random.Generator;
   Noise  : Fake_Camera.Noise_Source := Fake_Camera.Start_Noise (Seed);

   function Bits is new Ada.Unchecked_Conversion (Real, Unsigned_64);

   --  FNV-1a over 64 bits.
   Basis : constant Unsigned_64 := 14695981039346656037;
   Prime : constant Unsigned_64 := 1099511628211;

   type Byte_Access is access Driver.Bytes.Byte_Array;
   type Real_Access is access Real_Array;
   Pixels    : array (1 .. 16) of Byte_Access;
   Scratch   : array (1 .. 16) of Real_Access;
   Cam_State : array (1 .. 16) of Fake_Camera.State;
   Render_Time, Expose_Time : Duration := 0.0;
   Pictures : Natural := 0;
begin
   Shape.Servo := Option ("--servo", "no") = "yes";
   Shape.Class := (if Option ("--class") = "humanoid" then Humanoid elsif Option ("--class") = "base" then Base_And_Arm
                   elsif Option ("--class") = "drone" then Drone else Table_Arms);
   Build (W, Seed, Width, Height, Shape);
   Ada.Numerics.Float_Random.Reset (Draws, Seed);
   for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
      Pixels (C) := new Driver.Bytes.Byte_Array (1 .. Driver.Bytes.Offset (3 * Width * Height));
      Scratch (C) := new Real_Array (1 .. 3 * Width * Height);
      Cam_State (C) := Fake_Camera.Start (Seed, C, True, True, True);
   end loop;
   for Beat in 1 .. Beats loop
      if Beat mod Every = 1 then
         for G in W.Groups.First_Index .. W.Groups.Last_Index loop
            if W.Groups (G).Commandable then
               declare
                  Values : Real_Array := Readings (W, G);
               begin
                  for K in Values'Range loop
                     if K - Values'First < Natural (W.Groups (G).Joints.Length) then
                        declare
                           L : Link renames W.Links (W.Groups (G).Joints (K - Values'First + 1));
                           U : constant Real := Real (Ada.Numerics.Float_Random.Random (Draws));
                        begin
                           if L.Hi > L.Lo then
                              Values (K) := L.Lo + U * (L.Hi - L.Lo);
                           end if;
                        end;
                     end if;
                  end loop;
                  Command (W, G, Values);
               end;
            end if;
         end loop;
      end if;
      Step (W);
      for C in W.Cameras.First_Index .. W.Cameras.Last_Index loop
         declare
            Cam   : Camera renames W.Cameras (C);
            Share : constant Real := (if W.Quirk.Settle and then not Cam.Fresh then W.Quirk.Settle_Share else 1.0);
            T0    : constant Ada.Calendar.Time := Ada.Calendar.Clock;
            T1    : Ada.Calendar.Time;
            H_Render, H_Picture : Unsigned_64 := Basis;
         begin
            Fake_Camera.Next_Beat (Cam_State (C));
            if Fake_GPU.Available then
               --  Rendered, settled and exposed at once: its time all the render's.
               Fake_GPU.Picture (W, C, True, Share, Cam_State (C), Noise, Pixels (C).all);
               T1 := Ada.Calendar.Clock;
            else
               Fake_Render.Render (W, Cam, Scratch (C).all);
               T1 := Ada.Calendar.Clock;
               for I in Scratch (C)'Range loop
                  Cam.Shown (I) := (1.0 - Share) * Cam.Shown (I) + Share * Scratch (C) (I);
               end loop;
               Fake_Camera.Expose (Cam_State (C), Cam.Shown.all, Cam.Width, Cam.Height, W.Quirk.Image_Noise,
                                   Noise, Pixels (C).all);
            end if;
            Cam.Fresh := False;
            Render_Time := Render_Time + (T1 - T0);
            Expose_Time := Expose_Time + (Ada.Calendar.Clock - T1);
            Pictures := Pictures + 1;
            for V of Scratch (C).all loop
               H_Render := (H_Render xor Bits (V)) * Prime;
            end loop;
            for B of Pixels (C).all loop
               H_Picture := (H_Picture xor Unsigned_64 (B)) * Prime;
            end loop;
            Put_Line ("beat" & Beat'Image & " camera" & C'Image & " render" & H_Render'Image
                      & " picture" & H_Picture'Image);
            --  FAKE_FRAMES_DUMP names a folder: the bytes of every picture of the first, middle and last beats go there
            --  as they are sent (RGB, rows top to bottom), for comparing two ways of making them.
            if Ada.Environment_Variables.Exists ("FAKE_FRAMES_DUMP")
              and then (Beat = 1 or else Beat = Beats / 2 or else Beat = Beats)
            then
               declare
                  F : Ada.Streams.Stream_IO.File_Type;
               begin
                  Ada.Streams.Stream_IO.Create
                    (F, Ada.Streams.Stream_IO.Out_File, Ada.Environment_Variables.Value ("FAKE_FRAMES_DUMP") & "/beat"
                     & Ada.Strings.Fixed.Trim (Beat'Image, Ada.Strings.Left) & "_camera"
                     & Ada.Strings.Fixed.Trim (C'Image, Ada.Strings.Left) & ".raw");
                  Driver.Bytes.Byte_Array'Write (Ada.Streams.Stream_IO.Stream (F), Pixels (C).all);
                  Ada.Streams.Stream_IO.Close (F);
               end;
            end if;
         end;
      end loop;
   end loop;
   Put_Line ("pictures" & Pictures'Image
             & ", render" & Natural'Image (Natural (Real (Render_Time) * 1000.0 / Real (Pictures)))
             & " ms, settle and expose" & Natural'Image (Natural (Real (Expose_Time) * 1000.0 / Real (Pictures)))
             & " ms a picture");
end Fake_Frames;
