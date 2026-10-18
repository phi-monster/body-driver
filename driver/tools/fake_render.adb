with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Interfaces;
with Driver.Numerics;

package body Fake_Render is

   use Driver;
   use Driver.Numerics;
   use Ada.Numerics.Long_Elementary_Functions;
   use Fake_World;

   Light   : constant Vec3 := Unit ([0.3, -0.5, 0.8]);
   Ambient : constant := 0.35;
   Sky     : constant := 110.0;     --  what a ray that meets nothing shows
   Near    : constant := 1.0e-6;    --  a hit nearer than this is the ray's own surface
   Bands   : constant := 8;         --  rows are rendered in this many pieces at once

   --  A solid as one picture sees it, in plain scalars: the eye in the
   --  solid's frame and the rotation into it, made once a picture; the hot
   --  loop below touches no array operator, which would allocate.
   type Seen_From is record
      Rxx, Rxy, Rxz, Ryx, Ryy, Ryz, Rzx, Rzy, Rzz : Real := 0.0;   --  world direction into the solid's frame
      Ox, Oy, Oz : Real := 0.0;                                     --  the eye in the solid's frame
   end record;

   type Seen_Array is array (Positive range <>) of Seen_From;
   type Seen_Access is access Seen_Array;
   type Solid_Array is array (Positive range <>) of Solid;
   type Solid_Access is access Solid_Array;
   type Camera_Access is access constant Fake_World.Camera;
   type Pixels_Access is access all Driver.Real_Array;

   --  The picture being rendered, shared by the band workers: set by Render
   --  before the bands go, read until they are done.
   Seen   : Seen_Access;
   Solids : Solid_Access;
   Cam    : Camera_Access;
   Into   : Pixels_Access;

   function Hash (X, Y : Integer) return Real;
   function Smooth (T : Real) return Real;
   function Noise2 (X, Y : Real) return Real;
   function Shade_Of (M : Material; U, V : Real) return Real;
   function Distance (S : Solid; F : Seen_From; Dx, Dy, Dz : Real) return Real;
   procedure Rows (Band : Positive);

   --  The workers live as long as the program: a task made and ended for
   --  every band of every picture was what ended the fake robot after some
   --  twelve thousand beats on the lab's Mac (the system killed it), and the
   --  band of a picture is too small a piece of work to pay for one anyway.
   task type Band_Worker is
      entry Go (Band : Positive);
      entry Done;
   end Band_Worker;

   task body Band_Worker is
      Mine : Positive;
   begin
      loop
         select
            accept Go (Band : Positive) do
               Mine := Band;
            end Go;
         or
            terminate;
         end select;
         Rows (Mine);
         accept Done;
      end loop;
   end Band_Worker;

   Workers : array (2 .. Bands) of Band_Worker;

   --  A lattice value in [-1, 1] that looks random.
   function Hash (X, Y : Integer) return Real is
      use Interfaces;
      H : Unsigned_64 := Unsigned_64 (Unsigned_32'Mod (X)) * 374761393 + Unsigned_64 (Unsigned_32'Mod (Y)) * 668265263;
   begin
      H := (H xor Shift_Right (H, 13)) * 1274126177;
      H := H xor Shift_Right (H, 16);
      return Real (H mod 65536) / 32768.0 - 1.0;
   end Hash;

   function Smooth (T : Real) return Real is (T * T * T * (T * (T * 6.0 - 15.0) + 10.0));

   function Noise2 (X, Y : Real) return Real is
      X0 : constant Integer := Integer (Real'Floor (X));
      Y0 : constant Integer := Integer (Real'Floor (Y));
      Fx : constant Real := Smooth (X - Real (X0));
      Fy : constant Real := Smooth (Y - Real (Y0));
      A  : constant Real := Hash (X0, Y0);
      B  : constant Real := Hash (X0 + 1, Y0);
      C  : constant Real := Hash (X0, Y0 + 1);
      D  : constant Real := Hash (X0 + 1, Y0 + 1);
      Top    : constant Real := A + (B - A) * Fx;
      Bottom : constant Real := C + (D - C) * Fx;
   begin
      return Top + (Bottom - Top) * Fy;
   end Noise2;

   --  The texture's modulation of the albedo at a point (U, V) of the solid's own frame, in metres.
   function Shade_Of (M : Material; U, V : Real) return Real is
      Su : constant Real := U * M.Scale;
      Sv : constant Real := V * M.Scale;
      T  : Real;
   begin
      case M.Kind is
         when Flat =>
            return 1.0;
         when Noise =>
            T := 0.6 * Noise2 (Su, Sv) + 0.3 * Noise2 (2.0 * Su + 17.0, 2.0 * Sv + 3.0)
              + 0.1 * Noise2 (4.0 * Su + 5.0, 4.0 * Sv + 11.0);
         when Stripes =>
            T := Sin (2.0 * Ada.Numerics.Pi * Su) * 0.7 + 0.3 * Noise2 (3.0 * Su, 3.0 * Sv);
         when Grain =>
            --  Streaks along one axis that wander, two incommensurate families of them over noise at two scales, as
            --  wood does: one family of even streaks is a periodic texture, and a flow that tracks it aliases by
            --  a period whenever the picture moves more than half of one between two beats.
            T := 0.35 * Sin (2.0 * Ada.Numerics.Pi * (0.3 * Su + 0.8 * Noise2 (0.1 * Su, 0.6 * Sv)))
              + 0.25 * Sin (2.0 * Ada.Numerics.Pi * (0.73 * Su + 1.1 * Noise2 (0.17 * Su + 3.0, 0.4 * Sv)))
              + 0.4 * Noise2 (Su, 6.0 * Sv) + 0.3 * Noise2 (3.0 * Su + 7.0, 18.0 * Sv);
      end case;
      return 1.0 + M.Contrast * T;
   end Shade_Of;

   --  How far along a ray (world direction Dx, Dy, Dz, unit length) the solid
   --  is first met; Real'Last when it is not.
   function Distance (S : Solid; F : Seen_From; Dx, Dy, Dz : Real) return Real is
      --  The direction in the solid's frame.
      Lx : constant Real := F.Rxx * Dx + F.Rxy * Dy + F.Rxz * Dz;
      Ly : constant Real := F.Ryx * Dx + F.Ryy * Dy + F.Ryz * Dz;
      Lz : constant Real := F.Rzx * Dx + F.Rzy * Dy + F.Rzz * Dz;
   begin
      case S.Kind is
         when Plane =>
            if abs Lz < 1.0e-12 then
               return Real'Last;
            end if;
            declare
               T : constant Real := -F.Oz / Lz;
            begin
               return (if T > Near then T else Real'Last);
            end;
         when Box =>
            declare
               T_In  : Real := Real'First;
               T_Out : Real := Real'Last;
               procedure Slab (O, D, H : Real);
               procedure Slab (O, D, H : Real) is
               begin
                  if abs D < 1.0e-12 then
                     if abs O > H then
                        T_Out := Real'First;
                     end if;
                  else
                     declare
                        T1 : constant Real := (-H - O) / D;
                        T2 : constant Real := (H - O) / D;
                     begin
                        if T1 < T2 then
                           T_In := Real'Max (T_In, T1);
                           T_Out := Real'Min (T_Out, T2);
                        else
                           T_In := Real'Max (T_In, T2);
                           T_Out := Real'Min (T_Out, T1);
                        end if;
                     end;
                  end if;
               end Slab;
            begin
               Slab (F.Ox, Lx, S.Half (1));
               Slab (F.Oy, Ly, S.Half (2));
               Slab (F.Oz, Lz, S.Half (3));
               return (if T_In <= T_Out and then T_In > Near then T_In else Real'Last);
            end;
      end case;
   end Distance;

   --  The rows of one band of the picture being rendered.
   procedure Rows (Band : Positive) is
      C         : Fake_World.Camera renames Cam.all;
      First_Row : constant Natural := (Band - 1) * C.Height / Bands;
      Last_Row  : constant Natural := Band * C.Height / Bands - 1;
      R         : Mat3 renames C.Pose.Rotation;
   begin
      for Row in First_Row .. Last_Row loop
         for Col in 0 .. C.Width - 1 loop
            declare
               K   : constant Positive := Into'First + 3 * (Row * C.Width + Col);
               Acc : array (1 .. 3) of Real := [others => 0.0];
            begin
               for Sub_Y in 1 .. Supersample loop
                  for Sub_X in 1 .. Supersample loop
                     declare
                        --  The ray through the sub-sample's place in the pixel, in the camera's frame then the world's.
                        Px : constant Real := (Real (Col) + (Real (Sub_X) - 0.5) / Real (Supersample) - C.Cx) / C.Fx;
                        Py : constant Real := (Real (Row) + (Real (Sub_Y) - 0.5) / Real (Supersample) - C.Cy) / C.Fy;
                        Pn : constant Real := Sqrt (Px * Px + Py * Py + 1.0);
                        Cx : constant Real := Px / Pn;
                        Cy : constant Real := Py / Pn;
                        Cz : constant Real := 1.0 / Pn;
                        Dx : constant Real := R (1, 1) * Cx + R (1, 2) * Cy + R (1, 3) * Cz;
                        Dy : constant Real := R (2, 1) * Cx + R (2, 2) * Cy + R (2, 3) * Cz;
                        Dz : constant Real := R (3, 1) * Cx + R (3, 2) * Cy + R (3, 3) * Cz;
                        Best_T : Real := Real'Last;
                        Best_S : Natural := 0;
                     begin
                        for I in Seen'Range loop
                           declare
                              T : constant Real := Distance (Solids (I), Seen (I), Dx, Dy, Dz);
                           begin
                              if T < Best_T then
                                 Best_T := T;
                                 Best_S := I;
                              end if;
                           end;
                        end loop;
                        if Best_S = 0 then
                           Acc (1) := Acc (1) + Sky;
                           Acc (2) := Acc (2) + Sky;
                           Acc (3) := Acc (3) + Sky;
                        else
                           declare
                              S : Solid renames Solids (Best_S);
                              F : Seen_From renames Seen (Best_S);
                              --  The hit in the solid's frame, and its outward normal there.
                              Lx : constant Real := F.Rxx * Dx + F.Rxy * Dy + F.Rxz * Dz;
                              Ly : constant Real := F.Ryx * Dx + F.Ryy * Dy + F.Ryz * Dz;
                              Lz : constant Real := F.Rzx * Dx + F.Rzy * Dy + F.Rzz * Dz;
                              Hx : constant Real := F.Ox + Best_T * Lx;
                              Hy : constant Real := F.Oy + Best_T * Ly;
                              Hz : constant Real := F.Oz + Best_T * Lz;
                              Nx, Ny, Nz : Real := 0.0;
                              U, V : Real;
                           begin
                              if S.Kind = Plane then
                                 Nz := (if Lz < 0.0 then 1.0 else -1.0);
                                 U := Hx;
                                 V := Hy;
                              else
                                 --  The face is the one the hit lies on, to the rounding of the slab test.
                                 declare
                                    Ex : constant Real := abs (abs Hx - S.Half (1));
                                    Ey : constant Real := abs (abs Hy - S.Half (2));
                                    Ez : constant Real := abs (abs Hz - S.Half (3));
                                 begin
                                    if Ex <= Ey and then Ex <= Ez then
                                       Nx := (if Hx > 0.0 then 1.0 else -1.0);
                                       U := Hy;
                                       V := Hz;
                                    elsif Ey <= Ez then
                                       Ny := (if Hy > 0.0 then 1.0 else -1.0);
                                       U := Hx;
                                       V := Hz;
                                    else
                                       Nz := (if Hz > 0.0 then 1.0 else -1.0);
                                       U := Hx;
                                       V := Hy;
                                    end if;
                                 end;
                              end if;
                              declare
                                 --  The normal in the world (the rotation transposed).
                                 Wx : constant Real := F.Rxx * Nx + F.Ryx * Ny + F.Rzx * Nz;
                                 Wy : constant Real := F.Rxy * Nx + F.Ryy * Ny + F.Rzy * Nz;
                                 Wz : constant Real := F.Rxz * Nx + F.Ryz * Ny + F.Rzz * Nz;
                                 Diffuse : constant Real :=
                                   Real'Max (0.0, Wx * Light (1) + Wy * Light (2) + Wz * Light (3));
                                 --  Blinn-Phong: the half vector between the light and the way back to the eye.
                                 Hhx : constant Real := Light (1) - Dx;
                                 Hhy : constant Real := Light (2) - Dy;
                                 Hhz : constant Real := Light (3) - Dz;
                                 Hn  : constant Real := Sqrt (Hhx * Hhx + Hhy * Hhy + Hhz * Hhz);
                                 Spec_Cos : constant Real := Real'Max (0.0, (Wx * Hhx + Wy * Hhy + Wz * Hhz) / Hn);
                                 M       : Material renames S.Mat;
                                 Spec    : constant Real := (if M.Gloss > 0.0 then M.Gloss * Spec_Cos ** 40 else 0.0);
                                 Facing  : constant Real := 0.92 + 0.08 * abs (Wx * Dx + Wy * Dy + Wz * Dz);
                                 Tex     : constant Real := Shade_Of (M, U, V);
                                 Level   : constant Real := (Ambient + (1.0 - Ambient) * Diffuse) * Tex * Facing;
                              begin
                                 for Ch in 1 .. 3 loop
                                    Acc (Ch) := Acc (Ch)
                                      + Real'Max (0.0, Real'Min (255.0, 255.0 * (M.Albedo (Ch) * Level + Spec)));
                                 end loop;
                              end;
                           end;
                        end if;
                     end;
                  end loop;
               end loop;
               for Ch in 1 .. 3 loop
                  Into (K + Ch - 1) := Acc (Ch) / Real (Supersample * Supersample);
               end loop;
            end;
         end loop;
      end loop;
   end Rows;

   procedure Nearest
     (W : Fake_World.World; C : Fake_World.Camera; U, V : Driver.Real; Solid : out Natural; Along : out Driver.Real)
   is
      R  : Mat3 renames C.Pose.Rotation;
      Px : constant Real := (U - C.Cx) / C.Fx;
      Py : constant Real := (V - C.Cy) / C.Fy;
      Pn : constant Real := Sqrt (Px * Px + Py * Py + 1.0);
      Dx : constant Real := (R (1, 1) * Px + R (1, 2) * Py + R (1, 3)) / Pn;
      Dy : constant Real := (R (2, 1) * Px + R (2, 2) * Py + R (2, 3)) / Pn;
      Dz : constant Real := (R (3, 1) * Px + R (3, 2) * Py + R (3, 3)) / Pn;
   begin
      Solid := 0;
      Along := Real'Last;
      for I in W.Solids.First_Index .. W.Solids.Last_Index loop
         declare
            Inv : constant Rigid := Inverse (W.Solids (I).Pose);
            O   : constant Vec3 := Inv * C.Pose.Translation;
            F   : constant Seen_From :=
              (Rxx => Inv.Rotation (1, 1), Rxy => Inv.Rotation (1, 2), Rxz => Inv.Rotation (1, 3),
               Ryx => Inv.Rotation (2, 1), Ryy => Inv.Rotation (2, 2), Ryz => Inv.Rotation (2, 3),
               Rzx => Inv.Rotation (3, 1), Rzy => Inv.Rotation (3, 2), Rzz => Inv.Rotation (3, 3),
               Ox => O (1), Oy => O (2), Oz => O (3));
            T   : constant Real := Distance (W.Solids (I), F, Dx, Dy, Dz);
         begin
            if T < Along then
               Along := T;
               Solid := I;
            end if;
         end;
      end loop;
   end Nearest;

   procedure Render (W : Fake_World.World; C : aliased Fake_World.Camera; Pixels : aliased in out Driver.Real_Array) is
      Eye : constant Vec3 := C.Pose.Translation;
   begin
      if Seen = null or else Seen'Length /= Natural (W.Solids.Length) then
         Seen := new Seen_Array (W.Solids.First_Index .. W.Solids.Last_Index);
         Solids := new Solid_Array (W.Solids.First_Index .. W.Solids.Last_Index);
      end if;
      for I in Seen'Range loop
         Solids (I) := W.Solids (I);
         declare
            Inv : constant Rigid := Inverse (Solids (I).Pose);
            O   : constant Vec3 := Inv * Eye;
         begin
            Seen (I) := (Rxx => Inv.Rotation (1, 1), Rxy => Inv.Rotation (1, 2), Rxz => Inv.Rotation (1, 3),
                         Ryx => Inv.Rotation (2, 1), Ryy => Inv.Rotation (2, 2), Ryz => Inv.Rotation (2, 3),
                         Rzx => Inv.Rotation (3, 1), Rzy => Inv.Rotation (3, 2), Rzz => Inv.Rotation (3, 3),
                         Ox => O (1), Oy => O (2), Oz => O (3));
         end;
      end loop;
      Cam := C'Unchecked_Access;
      Into := Pixels'Unchecked_Access;
      for B in Workers'Range loop
         Workers (B).Go (B);
      end loop;
      Rows (1);
      for B in Workers'Range loop
         Workers (B).Done;
      end loop;
   end Render;

end Fake_Render;
