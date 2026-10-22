with Ada.Numerics;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Parallel_For;
with Fake_Rounding;

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
               Coef (K, L) := Fake_Rounding.Round (S / Q (K, L)) * Q (K, L);
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

   --  A plane of W x H samples, row by row, through the codec in 8x8 blocks, the picture's edge repeated past it: its
   --  rows of blocks at once, every block reading and writing its own samples only.
   procedure Code_Plane (Plane : in out Real_Array; W, H : Positive; Q : Table) is
      procedure Block_Row (Row : Positive) is
         By    : constant Natural := Row - 1;
         Block : Table;
      begin
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
      end Block_Row;
      procedure Block_Rows is new Driver.Parallel_For (Block_Row);
   begin
      Block_Rows (1, (H + 7) / 8);
   end Code_Plane;

   function Uniform (S : in out State) return Real is
   begin
      S.Rng := S.Rng xor Shift_Left (S.Rng, 13);
      S.Rng := S.Rng xor Shift_Right (S.Rng, 7);
      S.Rng := S.Rng xor Shift_Left (S.Rng, 17);
      return Real (S.Rng mod 2 ** 24) / Real (2 ** 24);
   end Uniform;

   --  One step of the sensors' noise generator (xorshift: 13, 7, 17).
   function Next (X : Unsigned_64) return Unsigned_64 is
      Y : constant Unsigned_64 := X xor Shift_Left (X, 13);
      Z : constant Unsigned_64 := Y xor Shift_Right (Y, 7);
   begin
      return Z xor Shift_Left (Z, 17);
   end Next;

   --  The noise's Gaussian: four of the generator's uniforms summed, centred and scaled to a unit variance.
   procedure Gauss (X : in out Unsigned_64; G : out Real) is
      S : Real := 0.0;
   begin
      for I in 1 .. 4 loop
         X := Next (X);
         S := S + Real (X mod 2 ** 24) / Real (2 ** 24);
      end loop;
      G := (S - 2.0) * 1.7320508075688772;
   end Gauss;

   --  A map of the generator's state that is linear over the field of two elements, as each of its steps is: the
   --  image of each of the state's bits. A picture's noise is drawn a row at a time on every processor at once, each
   --  row from where the draws of the rows before it leave the generator (Steps of their length), so that every pixel
   --  gets the noise the one stream drawn in order gives it.
   type Linear is array (0 .. 63) of Unsigned_64;

   function Apply (A : Linear; X : Unsigned_64) return Unsigned_64 is
      R : Unsigned_64 := 0;
      V : Unsigned_64 := X;
      B : Natural := 0;
   begin
      while V /= 0 loop
         if (V and 1) /= 0 then
            R := R xor A (B);
         end if;
         V := Shift_Right (V, 1);
         B := B + 1;
      end loop;
      return R;
   end Apply;

   --  B, then A.
   function After (A, B : Linear) return Linear is
      R : Linear;
   begin
      for I in R'Range loop
         R (I) := Apply (A, B (I));
      end loop;
      return R;
   end After;

   --  N steps of the generator.
   function Steps (N : Natural) return Linear is
      Power  : Linear;
      Result : Linear;
      K      : Natural := N;
   begin
      for I in Power'Range loop
         Power (I) := Next (Shift_Left (Unsigned_64'(1), I));
         Result (I) := Shift_Left (Unsigned_64'(1), I);
      end loop;
      while K > 0 loop
         if K mod 2 = 1 then
            Result := After (Power, Result);
         end if;
         Power := After (Power, Power);
         K := K / 2;
      end loop;
      return Result;
   end Steps;

   --  The steps a row of noise takes, kept for the row length they were made for.
   Row_Length : Natural := 0;
   Row_Steps  : Linear := [others => 0];

   function Start_Noise (Seed : Natural) return Noise_Source is
     ((State => Unsigned_64 (Seed) * 6364136223846793005 + 1442695040888963407));

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
      Noise         : in out Noise_Source;
      Picture       : out Driver.Bytes.Byte_Array)
   is
      N       : constant Positive := Width * Height;
      R, G, B : Real_Array (1 .. N);
      Mean    : Real := 0.0;
      Starts  : array (0 .. Height - 1) of Unsigned_64;   --  the generator where each row's noise begins
      function Clip (V : Real) return Real is (Real'Max (0.0, Real'Min (255.0, V)));

      --  Each row of pixels on its own: the noise, the picture's sums and the bytes are a pixel's, and a row's.
      procedure Exposed (Row : Positive) is
         X : Unsigned_64 := Starts (Row - 1);
         V : Real;
      begin
         for I in (Row - 1) * Width + 1 .. Row * Width loop
            declare
               K : constant Positive := Radiance'First + 3 * (I - 1);
            begin
               Gauss (X, V);
               R (I) := Clip (Radiance (K) * S.Gain + Noise_Level * V);
               Gauss (X, V);
               G (I) := Clip (Radiance (K + 1) * S.Gain + Noise_Level * V);
               Gauss (X, V);
               B (I) := Clip (Radiance (K + 2) * S.Gain + Noise_Level * V);
            end;
         end loop;
      end Exposed;
      procedure Exposed_Rows is new Driver.Parallel_For (Exposed);

      procedure Sent (Row : Positive) is
      begin
         for I in (Row - 1) * Width + 1 .. Row * Width loop
            declare
               use type Driver.Bytes.Offset;
               K : constant Driver.Bytes.Offset := Picture'First + Driver.Bytes.Offset (3 * (I - 1));
            begin
               Picture (K) := Driver.Bytes.Byte (Integer (Fake_Rounding.Floor (Clip (R (I)) + 0.5)));
               Picture (K + 1) := Driver.Bytes.Byte (Integer (Fake_Rounding.Floor (Clip (G (I)) + 0.5)));
               Picture (K + 2) := Driver.Bytes.Byte (Integer (Fake_Rounding.Floor (Clip (B (I)) + 0.5)));
            end;
         end loop;
      end Sent;
      procedure Sent_Rows is new Driver.Parallel_For (Sent);
   begin
      if Row_Length /= 12 * Width then
         Row_Steps := Steps (12 * Width);   --  three channels' Gaussians of four uniforms each, a pixel
         Row_Length := 12 * Width;
      end if;
      Starts (0) := Noise.State;
      for Row in 1 .. Height - 1 loop
         Starts (Row) := Apply (Row_Steps, Starts (Row - 1));
      end loop;
      Noise.State := Apply (Row_Steps, Starts (Height - 1));
      Exposed_Rows (1, Height);
      for I in 1 .. N loop
         Mean := Mean + 0.299 * R (I) + 0.587 * G (I) + 0.114 * B (I);
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

            --  JFIF's colour transform, the chroma averaged over 2 x 2 (4:2:0): a row of the chroma at a time, its
            --  rows of pixels in their order, as one pass over the picture sums them.
            procedure Transformed (Chroma_Row : Positive) is
            begin
               for Row in 2 * (Chroma_Row - 1) .. Natural'Min (Height - 1, 2 * Chroma_Row - 1) loop
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
               for J in (Chroma_Row - 1) * Cw + 1 .. Chroma_Row * Cw loop
                  Cb (J) := Cb (J) / Count (J);
                  Cr (J) := Cr (J) / Count (J);
               end loop;
            end Transformed;
            procedure Transformed_Rows is new Driver.Parallel_For (Transformed);

            --  Every plane coded, and back.
            procedure Decoded (Row_1 : Positive) is
               Row : constant Natural := Row_1 - 1;
            begin
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
            end Decoded;
            procedure Decoded_Rows is new Driver.Parallel_For (Decoded);
         begin
            Transformed_Rows (1, Ch);
            Code_Plane (Y, Width, Height, Scaled (Luma_Base, S.Quality));
            Code_Plane (Cb, Cw, Ch, Scaled (Chroma_Base, S.Quality));
            Code_Plane (Cr, Cw, Ch, Scaled (Chroma_Base, S.Quality));
            Decoded_Rows (1, Height);
         end;
      end if;
      Sent_Rows (1, Height);
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
