with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Interfaces;
with Driver.Numerics;
with Fake_Rounding;

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
   --  The rays' own mean place in a pixel, less its centre (Raw_Offset's over every sub-sample): taken off every
   --  ray, so that a pixel's light is about its centre. Set by Render before its workers start.
   Off_U, Off_V : Driver.Real := 0.0;

   --  A camera's rays, made once for it: where each of its pixels' rays meets the plane a unit in front of it
   --  (Fake_World.Ray at each sub-sample's place: undoing the lens takes Newton's steps, and they were made anew for
   --  every pixel of every picture), and the extent there of each tile's rays. Kept for a camera by its picture's
   --  size, its intrinsics and lens, and the rays a pixel takes with their places.
   Tile : constant := 16;   --  a tile's side, in pixels

   type Ray_Cache is record
      Width, Height, Samples : Natural := 0;
      Fx, Fy, Cx, Cy, K1, K2 : Real := 0.0;
      Blur                   : Real := 0.0;
      X, Y                   : Real_Access;   --  per pixel and sub-sample, rows top to bottom
      Lo_X, Hi_X, Lo_Y, Hi_Y : Real_Access;   --  per tile, rows of tiles top to bottom
   end record;
   Caches     : array (1 .. 16) of Ray_Cache;
   Cache      : Positive := 1;   --  the camera being rendered's
   Next_Cache : Positive := 1;   --  the one the next camera not kept takes

   --  Per tile of the picture being rendered, the solids its rays may meet, in their order (Render): tile after tile in
   --  Tile_List, the K-th tile's from Tile_First (K) to Tile_First (K + 1) less one.
   type Index_Array is array (Positive range <>) of Natural;
   type Index_Access is access Index_Array;
   Tile_List    : Index_Access;
   Tile_First   : Index_Access;
   Tiles_Across : Positive := 1;

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
      X0 : constant Integer := Integer (Fake_Rounding.Floor (X));
      Y0 : constant Integer := Integer (Fake_Rounding.Floor (Y));
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

   --  The place of a pixel's sub-sample Sub (from 0) along Axis (1 across, 2 down), in pixels from its corner: the R2
   --  sequence, the steps the inverse powers of the plastic number (the real root of x ** 3 = x + 1), the first at the
   --  pixel's centre.
   Plastic : constant Real := 1.324_717_957_244_746_025_960_908_854;

   function Lattice (Sub : Natural; Axis : Positive) return Real is
      Place : constant Real := 0.5 + Real (Sub) / Plastic ** Axis;
   begin
      return Place - Fake_Rounding.Floor (Place);
   end Lattice;

   --  With a lens blur, the sub-sample's place carries the lens's spread on top: the four-dimensional R2 sequence (the
   --  steps the inverse powers of the real root of x ** 5 = x + 1), its first two coordinates the place in the pixel
   --  and the last two a Gaussian offset of Lens_Blur pixels (Box and Muller), so that the rays fall as the pixel's
   --  area spread by the lens weighs the light.
   Quintic : constant Real := 1.167_303_978_261_418_684_256_045_899;

   function Spread (Sub : Natural; Axis : Positive) return Real is
      Place : constant Real := 0.5 + Real (Sub) / Quintic ** Axis;
   begin
      return Place - Fake_Rounding.Floor (Place);
   end Spread;

   function Raw_Offset (Sub : Natural; Axis : Positive) return Real is
   begin
      if Lens_Blur <= 0.0 then
         return Lattice (Sub, Axis);
      end if;
      declare
         Radius : constant Real :=
           Lens_Blur
           * Sqrt (-2.0 * Ada.Numerics.Long_Elementary_Functions.Log (Real'Max (Spread (Sub, 3), Real'Model_Small)));
         Turn   : constant Real := 2.0 * Ada.Numerics.Pi * Spread (Sub, 4);
      begin
         return Spread (Sub, Axis) + Radius * (if Axis = 1 then Cos (Turn) else Sin (Turn));
      end;
   end Raw_Offset;

   function Raw_Shift (Axis : Positive) return Driver.Real is
      Sum : Real := 0.0;
   begin
      for Sub in 0 .. Supersample * Supersample - 1 loop
         Sum := Sum + Raw_Offset (Sub, Axis);
      end loop;
      return Sum / Real (Supersample * Supersample) - 0.5;
   end Raw_Shift;

   function Offset (Sub : Natural; Axis : Positive) return Real is
     (Raw_Offset (Sub, Axis) - (if Axis = 1 then Off_U else Off_V));

   --  The rows of one band of the picture being rendered.
   procedure Rows (Band : Positive) is
      C         : Fake_World.Camera renames Cam.all;
      First_Row : constant Natural := (Band - 1) * C.Height / Bands;
      Last_Row  : constant Natural := Band * C.Height / Bands - 1;
      R         : Mat3 renames C.Pose.Rotation;
      Rays      : Ray_Cache renames Caches (Cache);
   begin
      for Row in First_Row .. Last_Row loop
         for Col in 0 .. C.Width - 1 loop
            declare
               K   : constant Positive := Into'First + 3 * (Row * C.Width + Col);
               Its_Tile : constant Positive := (Row / Tile) * Tiles_Across + Col / Tile + 1;
               Acc : array (1 .. 3) of Real := [others => 0.0];
            begin
               for Sub in 0 .. Supersample * Supersample - 1 loop
                  declare
                     --  The ray through the sub-sample's place in the pixel, in the camera's frame then the world's:
                     --  the places on the plastic number's lattice (Roberts' R2), every one of them at its own distance
                     --  along any line, so that an edge moving across the pixel changes it at as many places as there
                     --  are rays (a square grid of k by k crosses k of them at once when the edge lies along it). The
                     --  lens's distortion undone (Fake_World.Ray, made once for the camera: Caches): the light the
                     --  place shows.
                     N_Ray : constant Positive := (Row * C.Width + Col) * Rays.Samples + Sub + 1;
                     Px : constant Real := Rays.X (N_Ray);
                     Py : constant Real := Rays.Y (N_Ray);
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
                     for L in Tile_First (Its_Tile) .. Tile_First (Its_Tile + 1) - 1 loop
                        declare
                           I : constant Positive := Tile_List (L);
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
                                (if Even_Light then 1.0
                                 else Real'Max (0.0, Wx * Light (1) + Wy * Light (2) + Wz * Light (3)));
                              --  Blinn-Phong: the half vector between the light and the way back to the eye.
                              Hhx : constant Real := Light (1) - Dx;
                              Hhy : constant Real := Light (2) - Dy;
                              Hhz : constant Real := Light (3) - Dz;
                              Hn  : constant Real := Sqrt (Hhx * Hhx + Hhy * Hhy + Hhz * Hhz);
                              Spec_Cos : constant Real := Real'Max (0.0, (Wx * Hhx + Wy * Hhy + Wz * Hhz) / Hn);
                              M       : Material renames S.Mat;
                              Spec    : constant Real :=
                                (if not Even_Light and then M.Gloss > 0.0 then M.Gloss * Spec_Cos ** 40 else 0.0);
                              Facing  : constant Real :=
                                (if Even_Light then 1.0 else 0.92 + 0.08 * abs (Wx * Dx + Wy * Dy + Wz * Dz));
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
      Pr : constant Vec3 := Fake_World.Ray (C, U, V);
      Px : constant Real := Pr (1);
      Py : constant Real := Pr (2);
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
      Off_U := Raw_Shift (1);
      Off_V := Raw_Shift (2);
      --  The camera's rays, from those kept or made now.
      declare
         Samples : constant Positive := Supersample * Supersample;
         Found   : Natural := 0;
      begin
         for K in Caches'Range loop
            if Caches (K).Width = C.Width and then Caches (K).Height = C.Height and then Caches (K).Samples = Samples
              and then Caches (K).Fx = C.Fx and then Caches (K).Fy = C.Fy and then Caches (K).Cx = C.Cx
              and then Caches (K).Cy = C.Cy and then Caches (K).K1 = C.K1 and then Caches (K).K2 = C.K2
              and then Caches (K).Blur = Lens_Blur
            then
               Found := K;
            end if;
         end loop;
         if Found = 0 then
            Found := Next_Cache;
            Next_Cache := (if Next_Cache = Caches'Last then Caches'First else Next_Cache + 1);
            declare
               E      : Ray_Cache renames Caches (Found);
               Across : constant Positive := (C.Width + Tile - 1) / Tile;
               Down   : constant Positive := (C.Height + Tile - 1) / Tile;
            begin
               E := (Width => C.Width, Height => C.Height, Samples => Samples,
                     Fx => C.Fx, Fy => C.Fy, Cx => C.Cx, Cy => C.Cy, K1 => C.K1, K2 => C.K2, Blur => Lens_Blur,
                     X => new Real_Array (1 .. C.Width * C.Height * Samples),
                     Y => new Real_Array (1 .. C.Width * C.Height * Samples),
                     Lo_X => new Real_Array'(1 .. Across * Down => Real'Last),
                     Hi_X => new Real_Array'(1 .. Across * Down => Real'First),
                     Lo_Y => new Real_Array'(1 .. Across * Down => Real'Last),
                     Hi_Y => new Real_Array'(1 .. Across * Down => Real'First));
               for Row in 0 .. C.Height - 1 loop
                  for Col in 0 .. C.Width - 1 loop
                     for Sub in 0 .. Samples - 1 loop
                        declare
                           Pr : constant Vec3 :=
                             Fake_World.Ray (C, Real (Col) + Offset (Sub, 1), Real (Row) + Offset (Sub, 2));
                           N  : constant Positive := (Row * C.Width + Col) * Samples + Sub + 1;
                           T  : constant Positive := (Row / Tile) * Across + Col / Tile + 1;
                        begin
                           E.X (N) := Pr (1);
                           E.Y (N) := Pr (2);
                           E.Lo_X (T) := Real'Min (E.Lo_X (T), Pr (1));
                           E.Hi_X (T) := Real'Max (E.Hi_X (T), Pr (1));
                           E.Lo_Y (T) := Real'Min (E.Lo_Y (T), Pr (2));
                           E.Hi_Y (T) := Real'Max (E.Hi_Y (T), Pr (2));
                        end;
                     end loop;
                  end loop;
               end loop;
            end;
         end if;
         Cache := Found;
      end;
      --  Per tile, the solids its rays may meet: every plane; every box with a corner at or behind the plane of the
      --  eye; and every box all in front of it whose picture on the plane a unit in front (the hull of its corners'
      --  there: a perspective's image of a convex solid) meets the extent of the tile's rays there, widened by far
      --  more than the rounding of either. In their order, so that the nearest of them, and the first of equals, is
      --  the one the ray would meet among every solid.
      declare
         E      : Ray_Cache renames Caches (Cache);
         Tiles  : constant Positive := E.Lo_X'Length;
         Count  : Natural := 0;
         R      : Mat3 renames C.Pose.Rotation;
         Widen  : constant Real := 1.0e-9;   --  of the unit plane's extents, far beyond either's rounding
         type Extent is record
            Everywhere             : Boolean := True;
            Lo_X, Hi_X, Lo_Y, Hi_Y : Real := 0.0;
         end record;
         Extents : array (Seen'Range) of Extent;
      begin
         Tiles_Across := (C.Width + Tile - 1) / Tile;
         for I in Seen'Range loop
            if Solids (I).Kind = Box then
               declare
                  X : Extent := (Everywhere => False, Lo_X | Lo_Y => Real'Last, Hi_X | Hi_Y => Real'First);
               begin
                  for Ix in 0 .. 1 loop
                     for Iy in 0 .. 1 loop
                        for Iz in 0 .. 1 loop
                           declare
                              P  : constant Vec3 := Solids (I).Pose
                                * Vec3'[(if Ix = 0 then -Solids (I).Half (1) else Solids (I).Half (1)),
                                        (if Iy = 0 then -Solids (I).Half (2) else Solids (I).Half (2)),
                                        (if Iz = 0 then -Solids (I).Half (3) else Solids (I).Half (3))];
                              Wx : constant Real := P (1) - Eye (1);
                              Wy : constant Real := P (2) - Eye (2);
                              Wz : constant Real := P (3) - Eye (3);
                              --  The corner in the camera's frame (the rotation transposed).
                              Qx : constant Real := R (1, 1) * Wx + R (2, 1) * Wy + R (3, 1) * Wz;
                              Qy : constant Real := R (1, 2) * Wx + R (2, 2) * Wy + R (3, 2) * Wz;
                              Qz : constant Real := R (1, 3) * Wx + R (2, 3) * Wy + R (3, 3) * Wz;
                           begin
                              if Qz <= 0.0 then
                                 X.Everywhere := True;
                              else
                                 X.Lo_X := Real'Min (X.Lo_X, Qx / Qz);
                                 X.Hi_X := Real'Max (X.Hi_X, Qx / Qz);
                                 X.Lo_Y := Real'Min (X.Lo_Y, Qy / Qz);
                                 X.Hi_Y := Real'Max (X.Hi_Y, Qy / Qz);
                              end if;
                           end;
                        end loop;
                     end loop;
                  end loop;
                  if not X.Everywhere then
                     X.Lo_X := X.Lo_X - Widen * (1.0 + abs X.Lo_X);
                     X.Hi_X := X.Hi_X + Widen * (1.0 + abs X.Hi_X);
                     X.Lo_Y := X.Lo_Y - Widen * (1.0 + abs X.Lo_Y);
                     X.Hi_Y := X.Hi_Y + Widen * (1.0 + abs X.Hi_Y);
                  end if;
                  Extents (I) := X;
               end;
            end if;
         end loop;
         if Tile_List = null or else Tile_List'Length < Tiles * Seen'Length then
            Tile_List := new Index_Array (1 .. Tiles * Seen'Length);
         end if;
         if Tile_First = null or else Tile_First'Length < Tiles + 1 then
            Tile_First := new Index_Array (1 .. Tiles + 1);
         end if;
         for T in 1 .. Tiles loop
            Tile_First (T) := Count + 1;
            for I in Seen'Range loop
               if Extents (I).Everywhere
                 or else (Extents (I).Lo_X <= E.Hi_X (T) and then Extents (I).Hi_X >= E.Lo_X (T)
                          and then Extents (I).Lo_Y <= E.Hi_Y (T) and then Extents (I).Hi_Y >= E.Lo_Y (T))
               then
                  Count := Count + 1;
                  Tile_List (Count) := I;
               end if;
            end loop;
         end loop;
         Tile_First (Tiles + 1) := Count + 1;
      end;
      for B in Workers'Range loop
         Workers (B).Go (B);
      end loop;
      Rows (1);
      for B in Workers'Range loop
         Workers (B).Done;
      end loop;
   end Render;

end Fake_Render;
