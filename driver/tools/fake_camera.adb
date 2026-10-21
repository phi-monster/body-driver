with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;

package body Fake_Camera is

   use Ada.Numerics.Long_Elementary_Functions;
   use Interfaces;
   subtype Real_Array is Driver.Real_Array;

   Pi : constant := Ada.Numerics.Pi;

   --  The JPEG standard's example tables (ITU-T T.81, Annex K), luminance and chrominance, in natural order, which
   --  every webcam's encoder scales by its quality as libjpeg does.
   type Table is array (0 .. 7, 0 .. 7) of Real;

   Luma_Base : constant Table :=
     [[16.0, 11.0, 10.0, 16.0, 24.0, 40.0, 51.0, 61.0],
      [12.0, 12.0, 14.0, 19.0, 26.0, 58.0, 60.0, 55.0],
      [14.0, 13.0, 16.0, 24.0, 40.0, 57.0, 69.0, 56.0],
      [14.0, 17.0, 22.0, 29.0, 51.0, 87.0, 80.0, 62.0],
      [18.0, 22.0, 37.0, 56.0, 68.0, 109.0, 103.0, 77.0],
      [24.0, 35.0, 55.0, 64.0, 81.0, 104.0, 113.0, 92.0],
      [49.0, 64.0, 78.0, 87.0, 103.0, 121.0, 120.0, 101.0],
      [72.0, 92.0, 95.0, 98.0, 112.0, 100.0, 103.0, 99.0]];

   Chroma_Base : constant Table :=
     [[17.0, 18.0, 24.0, 47.0, 99.0, 99.0, 99.0, 99.0],
      [18.0, 21.0, 26.0, 66.0, 99.0, 99.0, 99.0, 99.0],
      [24.0, 26.0, 56.0, 99.0, 99.0, 99.0, 99.0, 99.0],
      [47.0, 66.0, 99.0, 99.0, 99.0, 99.0, 99.0, 99.0],
      [99.0, 99.0, 99.0, 99.0, 99.0, 99.0, 99.0, 99.0],
      [99.0, 99.0, 99.0, 99.0, 99.0, 99.0, 99.0, 99.0],
      [99.0, 99.0, 99.0, 99.0, 99.0, 99.0, 99.0, 99.0],
      [99.0, 99.0, 99.0, 99.0, 99.0, 99.0, 99.0, 99.0]];

   --  libjpeg's quality scaling: a step of the table no smaller than one and no larger than 255.
   function Scaled (Base : Table; Quality : Natural) return Table is
      Factor : constant Real := (if Quality < 50 then 5000.0 / Real (Natural'Max (1, Quality))
                                 else 200.0 - 2.0 * Real (Quality));
      R      : Table;
   begin
      for U in 0 .. 7 loop
         for V in 0 .. 7 loop
            R (U, V) := Real'Max (1.0, Real'Min (255.0, Real'Floor ((Base (U, V) * Factor + 50.0) / 100.0)));
         end loop;
      end loop;
      return R;
   end Scaled;

   --  The DCT's basis: C (K, N) = c(K) cos((2N + 1) K pi / 16), c(0) = sqrt(1/8), else sqrt(2/8).
   type Basis is array (0 .. 7, 0 .. 7) of Real;

   function Make_Basis return Basis is
      B : Basis;
   begin
      for K in 0 .. 7 loop
         for N in 0 .. 7 loop
            B (K, N) := (if K = 0 then Sqrt (1.0 / 8.0) else Sqrt (2.0 / 8.0))
              * Cos (Real (2 * N + 1) * Real (K) * Pi / 16.0);
         end loop;
      end loop;
      return B;
   end Make_Basis;

   C : constant Basis := Make_Basis;

   --  One 8x8 block through the codec: forward DCT, quantized by Q, back.
   procedure Code_Block (Block : in out Table; Q : Table) is
      Tmp, Coef : Table;
   begin
      for Y in 0 .. 7 loop
         for K in 0 .. 7 loop
            declare
               S : Real := 0.0;
            begin
               for X in 0 .. 7 loop
                  S := S + C (K, X) * Block (Y, X);
               end loop;
               Tmp (Y, K) := S;
            end;
         end loop;
      end loop;
      for K in 0 .. 7 loop
         for L in 0 .. 7 loop
            declare
               S : Real := 0.0;
            begin
               for Y in 0 .. 7 loop
                  S := S + C (K, Y) * Tmp (Y, L);
               end loop;
               --  The quantization: the coefficient kept to a whole number of its step.
               Coef (K, L) := Real'Rounding (S / Q (K, L)) * Q (K, L);
            end;
         end loop;
      end loop;
      for Y in 0 .. 7 loop
         for L in 0 .. 7 loop
            declare
               S : Real := 0.0;
            begin
               for K in 0 .. 7 loop
                  S := S + C (K, Y) * Coef (K, L);
               end loop;
               Tmp (Y, L) := S;
            end;
         end loop;
      end loop;
      for Y in 0 .. 7 loop
         for X in 0 .. 7 loop
            declare
               S : Real := 0.0;
            begin
               for L in 0 .. 7 loop
                  S := S + C (L, X) * Tmp (Y, L);
               end loop;
               Block (Y, X) := S;
            end;
         end loop;
      end loop;
   end Code_Block;

   --  A plane of W x H samples, row by row, through the codec in 8x8 blocks, the picture's edge repeated past it.
   procedure Code_Plane (Plane : in out Real_Array; W, H : Positive; Q : Table) is
      Block : Table;
   begin
      for By in 0 .. (H + 7) / 8 - 1 loop
         for Bx in 0 .. (W + 7) / 8 - 1 loop
            for Y in 0 .. 7 loop
               for X in 0 .. 7 loop
                  Block (Y, X) := Plane (Natural'Min (H - 1, By * 8 + Y) * W + Natural'Min (W - 1, Bx * 8 + X) + 1) - 128.0;
               end loop;
            end loop;
            Code_Block (Block, Q);
            for Y in 0 .. 7 loop
               for X in 0 .. 7 loop
                  if By * 8 + Y < H and then Bx * 8 + X < W then
                     Plane ((By * 8 + Y) * W + Bx * 8 + X + 1) := Block (Y, X) + 128.0;
                  end if;
               end loop;
            end loop;
         end loop;
      end loop;
   end Code_Plane;

   function Uniform (S : in out State) return Real is
   begin
      S.Rng := S.Rng xor Shift_Left (S.Rng, 13);
      S.Rng := S.Rng xor Shift_Right (S.Rng, 7);
      S.Rng := S.Rng xor Shift_Left (S.Rng, 17);
      return Real (S.Rng mod 2 ** 24) / Real (2 ** 24);
   end Uniform;

   function Start (Seed : Natural; Index : Positive; Exposure, Mjpeg, Gaps : Boolean) return State is
      S : State;
   begin
      S.Seed := Seed;
      S.Index := Index;
      S.Exposure := Exposure;
      S.Mjpeg := Mjpeg;
      S.Gaps := Gaps;
      S.Rng := Unsigned_64 (Seed) * 2654435761 + Unsigned_64 (Index) * 40503 + 12345;
      for Warm in 1 .. 4 loop
         declare
            Ignored : constant Real := Uniform (S);
         begin
            null;
         end;
      end loop;
      --  A webcam's MJPEG quality, from a rough one to a fine one.
      S.Quality := 60 + Natural (Real'Floor (30.0 * Uniform (S)));
      --  It starts half a stop to a stop off, either way, and finds its exposure over its first frames.
      declare
         Stops : constant Real := 0.5 + 0.5 * Uniform (S);
      begin
         S.Gain := (if Uniform (S) < 0.5 then 2.0 ** Stops else 2.0 ** (-Stops));
      end;
      if not Exposure then
         S.Gain := 1.0;
      end if;
      --  Hiccups: about one in one to three thousand beats, of a beat to a few seconds.
      S.Gap_Rate := 1.0 / (1000.0 + 2000.0 * Uniform (S));
      S.Gap_Most := 1 + Natural (Real'Floor (29.0 * Uniform (S)));
      return S;
   end Start;

   procedure Expose
     (S             : in out State;
      Radiance      : Driver.Real_Array;
      Width, Height : Positive;
      Noise_Level   : Real;
      Noise         : not null access function return Driver.Real;
      Picture       : out Driver.Bytes.Byte_Array)
   is
      N     : constant Positive := Width * Height;
      R, G, B : Real_Array (1 .. N);
      Mean  : Real := 0.0;
      function Clip (V : Real) return Real is (Real'Max (0.0, Real'Min (255.0, V)));
   begin
      for I in 1 .. N loop
         declare
            K : constant Positive := Radiance'First + 3 * (I - 1);
         begin
            R (I) := Clip (Radiance (K) * S.Gain + Noise_Level * Noise.all);
            G (I) := Clip (Radiance (K + 1) * S.Gain + Noise_Level * Noise.all);
            B (I) := Clip (Radiance (K + 2) * S.Gain + Noise_Level * Noise.all);
            Mean := Mean + 0.299 * R (I) + 0.587 * G (I) + 0.114 * B (I);
         end;
      end loop;
      Mean := Mean / Real (N);
      --  The exposure for the next frame: towards a mid grey, a third of a stop at most, once off by a sixth.
      if S.Exposure and then Mean > 0.0 then
         declare
            Off : constant Real := Log (118.0 / Mean, 2.0);
         begin
            if abs Off > 1.0 / 6.0 then
               S.Gain := S.Gain * 2.0 ** Real'Max (-1.0 / 3.0, Real'Min (1.0 / 3.0, Off));
            end if;
         end;
      end if;
      if S.Mjpeg then
         declare
            Cw : constant Positive := (Width + 1) / 2;
            Ch : constant Positive := (Height + 1) / 2;
            Y  : Real_Array (1 .. N);
            Cb, Cr : Real_Array (1 .. Cw * Ch) := [others => 0.0];
            Count  : Real_Array (1 .. Cw * Ch) := [others => 0.0];
         begin
            --  JFIF's colour transform, the chroma averaged over 2 x 2 (4:2:0), every plane coded, and back.
            for Row in 0 .. Height - 1 loop
               for Col in 0 .. Width - 1 loop
                  declare
                     I : constant Positive := Row * Width + Col + 1;
                     J : constant Positive := (Row / 2) * Cw + Col / 2 + 1;
                  begin
                     Y (I) := 0.299 * R (I) + 0.587 * G (I) + 0.114 * B (I);
                     Cb (J) := Cb (J) + 128.0 - 0.168736 * R (I) - 0.331264 * G (I) + 0.5 * B (I);
                     Cr (J) := Cr (J) + 128.0 + 0.5 * R (I) - 0.418688 * G (I) - 0.081312 * B (I);
                     Count (J) := Count (J) + 1.0;
                  end;
               end loop;
            end loop;
            for J in Cb'Range loop
               Cb (J) := Cb (J) / Count (J);
               Cr (J) := Cr (J) / Count (J);
            end loop;
            Code_Plane (Y, Width, Height, Scaled (Luma_Base, S.Quality));
            Code_Plane (Cb, Cw, Ch, Scaled (Chroma_Base, S.Quality));
            Code_Plane (Cr, Cw, Ch, Scaled (Chroma_Base, S.Quality));
            for Row in 0 .. Height - 1 loop
               for Col in 0 .. Width - 1 loop
                  declare
                     I  : constant Positive := Row * Width + Col + 1;
                     J  : constant Positive := (Row / 2) * Cw + Col / 2 + 1;
                     Pb : constant Real := Cb (J) - 128.0;
                     Pr : constant Real := Cr (J) - 128.0;
                  begin
                     R (I) := Y (I) + 1.402 * Pr;
                     G (I) := Y (I) - 0.344136 * Pb - 0.714136 * Pr;
                     B (I) := Y (I) + 1.772 * Pb;
                  end;
               end loop;
            end loop;
         end;
      end if;
      for I in 1 .. N loop
         declare
            use type Driver.Bytes.Offset;
            K : constant Driver.Bytes.Offset := Picture'First + Driver.Bytes.Offset (3 * (I - 1));
         begin
            Picture (K) := Driver.Bytes.Byte (Integer (Real'Floor (Clip (R (I)) + 0.5)));
            Picture (K + 1) := Driver.Bytes.Byte (Integer (Real'Floor (Clip (G (I)) + 0.5)));
            Picture (K + 2) := Driver.Bytes.Byte (Integer (Real'Floor (Clip (B (I)) + 0.5)));
         end;
      end loop;
   end Expose;

   procedure Next_Beat (S : in out State) is
   begin
      if not S.Gaps then
         return;
      end if;
      if S.Gap_Left > 0 then
         S.Gap_Left := S.Gap_Left - 1;
      else
         declare
            Begins : constant Real := Uniform (S);
            Length : constant Real := Uniform (S);
         begin
            if Begins < S.Gap_Rate then
               S.Gap_Left := 1 + Natural (Real'Floor (Real (S.Gap_Most) * Length));
            end if;
         end;
      end if;
   end Next_Beat;

   function Gives (S : State) return Boolean is (S.Gap_Left = 0);

   function Describe (S : State) return String is
     ("camera" & S.Index'Image & ": MJPEG " & (if S.Mjpeg then "quality" & S.Quality'Image else "off")
      & ", exposure " & (if S.Exposure then "starting at" & Natural'Image (Natural (100.0 * S.Gain)) & " %" else "fixed")
      & ", hiccups " & (if S.Gaps then "up to" & S.Gap_Most'Image & " beats, one in about"
                        & Natural'Image (Natural (1.0 / S.Gap_Rate)) & " beats" else "none"));

end Fake_Camera;
