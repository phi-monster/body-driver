with Ada.Environment_Variables;
with Ada.Unchecked_Conversion;
with Interfaces.C;
with Interfaces.C.Strings;
with System;
with Driver.Numerics;
with Fake_Render;

package body Fake_GPU is

   use Interfaces.C;
   use Driver;
   use Driver.Numerics;
   use type System.Address;
   use type Fake_World.Shape_Kind;

   type Float3 is array (0 .. 2) of C_float with Convention => C;
   type Float9 is array (0 .. 8) of C_float with Convention => C;
   type Float64 is array (0 .. 63) of C_float with Convention => C;

   --  fake_gpu.cu's fg_solid.
   type Solid is record
      Kind, Tex              : int := 0;
      Half                   : Float3 := [others => 0.0];
      Rot                    : Float9 := [others => 0.0];
      O                      : Float3 := [others => 0.0];
      Albedo                 : Float3 := [others => 0.0];
      Contrast, Scale, Gloss : C_float := 0.0;
   end record with Convention => C;
   type Solid_Array is array (Positive range <>) of Solid with Convention => C;

   type Init_Fn is access function (Which : int) return int with Convention => C;
   type Camera_Fn is access function (Cam, Width, Height : int; Ray : System.Address) return int with Convention => C;
   type Frame_Fn is access function
     (Cam, Render : int; Rot : System.Address; N : int; Solids : System.Address; Share, Gain, Noise_Level : C_float;
      Seed : unsigned_long_long; Mjpeg : int; Q_Luma, Q_Chroma : System.Address; Bytes : System.Address;
      Mean : access double) return int with Convention => C;
   function To_Init is new Ada.Unchecked_Conversion (System.Address, Init_Fn);
   function To_Camera is new Ada.Unchecked_Conversion (System.Address, Camera_Fn);
   function To_Frame is new Ada.Unchecked_Conversion (System.Address, Frame_Fn);

   function Dlopen (Path : Interfaces.C.Strings.chars_ptr; Mode : int) return System.Address;
   pragma Import (C, Dlopen, "dlopen");
   function Dlsym (Handle : System.Address; Name : Interfaces.C.Strings.chars_ptr) return System.Address;
   pragma Import (C, Dlsym, "dlsym");
   function Getpid return int;
   pragma Import (C, Getpid, "getpid");

   Tried, Ready : Boolean := False;
   Init  : Init_Fn;
   Cam   : Camera_Fn;
   Frame : Frame_Fn;
   Known : array (1 .. 16) of Boolean := [others => False];   --  the cameras the library has rays for

   function Available return Boolean is
   begin
      if not Tried then
         Tried := True;
         if Ada.Environment_Variables.Exists ("FAKE_GPU") then
            declare
               use Interfaces.C.Strings;
               Path   : chars_ptr := New_String (Ada.Environment_Variables.Value ("FAKE_GPU"));
               Lazy   : constant int := 1;   --  RTLD_LAZY
               Handle : constant System.Address := Dlopen (Path, Lazy);
            begin
               Free (Path);
               if Handle /= System.Null_Address then
                  declare
                     N1 : chars_ptr := New_String ("fg_init");
                     N2 : chars_ptr := New_String ("fg_camera");
                     N3 : chars_ptr := New_String ("fg_frame");
                     A1 : constant System.Address := Dlsym (Handle, N1);
                     A2 : constant System.Address := Dlsym (Handle, N2);
                     A3 : constant System.Address := Dlsym (Handle, N3);
                  begin
                     Free (N1);
                     Free (N2);
                     Free (N3);
                     if A1 /= System.Null_Address and then A2 /= System.Null_Address and then A3 /= System.Null_Address
                     then
                        Init := To_Init (A1);
                        Cam := To_Camera (A2);
                        Frame := To_Frame (A3);
                        --  The fakes on a machine spread over its devices by their process numbers.
                        Ready := Init (Getpid) = 0;
                     end if;
                  end;
               end if;
            end;
         end if;
      end if;
      --  The library makes what the renderer makes with one ray a pixel through its centre, no lens blur, the light
      --  as it falls; anything else stays with Fake_Render.
      return Ready and then Fake_Render.Supersample = 1 and then Fake_Render.Lens_Blur = 0.0
        and then not Fake_Render.Even_Light;
   end Available;

   procedure Picture
     (W           : Fake_World.World;
      Index       : Positive;
      Render      : Boolean;
      Share       : Driver.Real;
      S           : in out Fake_Camera.State;
      Noise       : in out Fake_Camera.Noise_Source;
      Bytes       : out Driver.Bytes.Byte_Array)
   is
      C      : Fake_World.Camera renames W.Cameras (Index);
      Rot    : Float9;
      Solids : Solid_Array (1 .. Natural'Max (1, Natural (W.Solids.Length)));
      Mean   : aliased double := 0.0;
      Q_Luma, Q_Chroma : Float64;
      Status : int;
   begin
      if not Known (Index) then
         declare
            type Ray_Table is array (Natural range <>) of C_float with Convention => C;
            Rays : Ray_Table (0 .. 2 * C.Width * C.Height - 1);
         begin
            for Row in 0 .. C.Height - 1 loop
               for Col in 0 .. C.Width - 1 loop
                  declare
                     P : constant Vec3 := Fake_World.Ray (C, Real (Col) + 0.5, Real (Row) + 0.5);
                     K : constant Natural := 2 * (Row * C.Width + Col);
                  begin
                     Rays (K) := C_float (P (1));
                     Rays (K + 1) := C_float (P (2));
                  end;
               end loop;
            end loop;
            if Cam (int (Index - 1), int (C.Width), int (C.Height), Rays'Address) /= 0 then
               raise Program_Error with "fake_gpu: camera" & Index'Image & " not set up";
            end if;
            Known (Index) := True;
         end;
      end if;
      for I in 1 .. 3 loop
         for J in 1 .. 3 loop
            Rot (3 * (I - 1) + J - 1) := C_float (C.Pose.Rotation (I, J));
         end loop;
      end loop;
      for K in W.Solids.First_Index .. W.Solids.Last_Index loop
         declare
            Sd  : Fake_World.Solid renames W.Solids (K);
            Inv : constant Rigid := Inverse (Sd.Pose);
            O   : constant Vec3 := Inv * C.Pose.Translation;
            G   : Solid renames Solids (K - W.Solids.First_Index + 1);
         begin
            G.Kind := (if Sd.Kind = Fake_World.Plane then 1 else 0);
            G.Tex := int (Fake_World.Texture_Kind'Pos (Sd.Mat.Kind));
            for I in 1 .. 3 loop
               G.Half (I - 1) := C_float (Sd.Half (I));
               G.O (I - 1) := C_float (O (I));
               G.Albedo (I - 1) := C_float (Sd.Mat.Albedo (I));
               for J in 1 .. 3 loop
                  G.Rot (3 * (I - 1) + J - 1) := C_float (Inv.Rotation (I, J));
               end loop;
            end loop;
            G.Contrast := C_float (Sd.Mat.Contrast);
            G.Scale := C_float (Sd.Mat.Scale);
            G.Gloss := C_float (Sd.Mat.Gloss);
         end;
      end loop;
      declare
         L : constant Fake_Camera.Step_Table := Fake_Camera.Luma_Steps (S);
         Ch : constant Fake_Camera.Step_Table := Fake_Camera.Chroma_Steps (S);
      begin
         for K in 0 .. 63 loop
            Q_Luma (K) := C_float (L (K));
            Q_Chroma (K) := C_float (Ch (K));
         end loop;
      end;
      Status := Frame
        (int (Index - 1), (if Render then 1 else 0), Rot'Address, int (W.Solids.Length), Solids'Address,
         C_float (Share), C_float (Fake_Camera.Gain (S)), C_float (W.Quirk.Image_Noise),
         unsigned_long_long (Fake_Camera.Draw_Seed (Noise)), (if Fake_Camera.Coded (S) then 1 else 0),
         Q_Luma'Address, Q_Chroma'Address, Bytes'Address, Mean'Access);
      if Status /= 0 then
         raise Program_Error with "fake_gpu: camera" & Index'Image & " picture failed," & Status'Image;
      end if;
      Fake_Camera.Adapt (S, Real (Mean));
   end Picture;

end Fake_GPU;
