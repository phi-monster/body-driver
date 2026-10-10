with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Alignment.Keyframes;
with Driver.Bytes;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Images;
with Driver.Parallel_For;
with Driver.Tests;
with Driver.Uncertain;

package body Driver.Alignment.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;
   use type Keyframes.Source;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      --  Random may return one itself (its range is closed), which would take the logarithm of zero.
      U1 : constant Real := Real'Max (1.0E-12, 1.0 - Real (Ada.Numerics.Float_Random.Random (Gen)));
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   Width  : constant := 320;
   Height : constant := 240;

   type Scene is (Textured, Flat, Edge, Periodic, Grain);

   type Wave is record
      Fu, Fv, Amplitude, Phase : Real;
   end record;

   --  Waves of wavelengths between seven and fourteen pixels at frequencies no two of which are multiples of one
   --  another, so the pattern repeats nowhere in the picture.
   Waves : constant array (1 .. 8) of Wave :=
     [(0.071, 0.043, 14.0, 0.3), (-0.052, 0.088, 11.0, 1.9), (0.113, -0.029, 9.0, 4.1), (0.027, 0.131, 8.0, 2.2),
      (-0.094, -0.067, 12.0, 5.5), (0.149, 0.052, 6.0, 0.8), (0.038, -0.104, 10.0, 3.3), (-0.126, 0.091, 7.0, 1.1)];

   --  The scene's brightness at a place of the scene.
   function Brightness (Kind : Scene; X, Y : Real) return Real is
      Pi : constant Real := Ada.Numerics.Pi;
   begin
      case Kind is
         when Textured =>
            declare
               Sum : Real := 128.0;
            begin
               for W of Waves loop
                  Sum := Sum + W.Amplitude * Sin (2.0 * Pi * (W.Fu * X + W.Fv * Y) + W.Phase);
               end loop;
               return Sum;
            end;
         when Flat =>
            return 100.0;
         when Edge =>
            --  A vertical edge, blurred over a few pixels, and nothing else.
            return 100.0 + 60.0 * (1.0 + Tanh ((X - 160.0) / 2.0));
         when Periodic =>
            --  The same every five pixels each way.
            return 128.0 + 50.0 * Sin (2.0 * Pi * X / 5.0) + 50.0 * Sin (2.0 * Pi * Y / 5.0);
         when Grain =>
            --  Lines about three pixels apart, slanting, each line of a strength of its own (as the lines of a grain
            --  are) and swelling slowly along its length: a pattern whose period the pixels sample only thrice.
            declare
               Phase : constant Real := 0.27 * X + 0.20 * Y;
               Line  : constant Real := Real'Floor (Phase);
            begin
               return 128.0 + (30.0 + 15.0 * Sin (0.73 * Line * Line + 1.3))
                 * (1.0 + 0.3 * Sin (2.0 * Pi * (0.6 * X - 0.8 * Y) / 14.0)) * Sin (2.0 * Pi * Phase);
            end;
      end case;
   end Brightness;

   --  How far a bend of the given amplitude carries a place of the scene along u: a wave of eighty pixels.
   function Bend_Of (X, Bend : Real) return Real is (Bend * Sin (2.0 * Ada.Numerics.Pi * X / 80.0));

   --  The place of the scene that the place (Du, Dv) of a picture, less the shift, shows through the linear part
   --  of a map bent by a wave: the unbent place, found again by iteration (the wave's slope is far under one).
   function Scene_Place (Linear : Linear_Part; Det, Du, Dv, Bend : Real) return Driver.Images.Pixel is
      X : Real := (Linear.VV * Du - Linear.UV * Dv) / Det;
      Y : Real := (-Linear.VU * Du + Linear.UU * Dv) / Det;
   begin
      for Pass in 1 .. 8 loop
         exit when Bend = 0.0;
         declare
            Bent : constant Real := Du - Bend_Of (X, Bend);
         begin
            X := (Linear.VV * Bent - Linear.UV * Dv) / Det;
            Y := (-Linear.VU * Bent + Linear.UU * Dv) / Det;
         end;
      end loop;
      return (U => X, V => Y);
   end Scene_Place;

   --  The picture of the scene seen through a map: the place (u', v') of the picture shows the scene at the
   --  place the map takes (u', v') from, (u', v') = Linear (x, y) + Shift; to the right of Split the shift is
   --  Right_Shift instead (a surface in front that moves otherwise). The map may be bent along u by a wave of the
   --  given amplitude (Bend_Of), which no affine map holds, and the light may change as a whole (a gain and an
   --  offset). Rounded to whole levels, with Gaussian noise of the given sigma first.
   function Picture
     (Kind : Scene; Linear : Linear_Part; Shift_U, Shift_V : Real; Noise : Real;
      Split : Real := Real'Last; Right_Shift : Real := 0.0; Bend : Real := 0.0; Light_Gain : Real := 1.0;
      Light_Offset : Real := 0.0) return Driver.Images.Image
   is
      use Driver.Bytes;
      use type Driver.Bytes.Offset;
      Data : Byte_Array (1 .. 3 * Width * Height);
      Det  : constant Real := Linear.UU * Linear.VV - Linear.UV * Linear.VU;
   begin
      for Row in 0 .. Height - 1 loop
         for Column in 0 .. Width - 1 loop
            declare
               Here : constant Real := Real (Column) + 0.5;
               Du : constant Real := Here - (if Here >= Split then Right_Shift else Shift_U);
               Dv : constant Real := Real (Row) + 0.5 - Shift_V;
               Place : constant Driver.Images.Pixel := Scene_Place (Linear, Det, Du, Dv, Bend);
               V  : constant Real :=
                 Brightness (Kind, Place.U, Place.V) * Light_Gain + Light_Offset + Noise * Gaussian;
               L  : constant Byte := Byte (Integer'Max (0, Integer'Min (255, Integer (Real'Rounding (V)))));
               K  : constant Offset := Offset (3 * (Row * Width + Column));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Width, Height, Data);
   end Picture;

   function Moved
     (Linear : Linear_Part; Shift_U, Shift_V : Real; P : Driver.Images.Pixel; Bend : Real := 0.0)
      return Driver.Images.Pixel is
     ((U => Linear.UU * P.U + Linear.UV * P.V + Shift_U + Bend_Of (P.U, Bend),
       V => Linear.VU * P.U + Linear.VV * P.V + Shift_V));

   function Sigma_Of (Variance : Real) return Real is (Sqrt (Real'Max (0.0, Variance)));

   --  How many standard deviations a difference must be to be significant for a covariance that rests on that
   --  many other fits (Driver.Uncertain's rule: Student's t at the tail probability Z has for a Gaussian).
   function Gate_Of (Degrees : Natural) return Real is
     (if Degrees = 0 then Driver.Conventions.Z
      else Driver.Distributions.Student_T_Quantile
        (Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z), Degrees));

   type Outcome is record
      Asked, Found, Wrong : Natural := 0;
      Worst               : Real := 0.0;       --  the largest distance of an answer from the truth
      Worst_Linear        : Real := 0.0;       --  the largest error of an entry of the linear part
      Axes, Within_Gate   : Natural := 0;      --  the axes of the found answers, and those within their own gate
      Narrowest, Widest   : Real := 0.0;       --  of the standard deviations, over the axes
      First_Reason        : Reason := Matched;
   end record;

   --  Aligns every grid point of one picture to the other, the prediction being the truth moved by Miss with the
   --  given standard deviation. Points whose truth is the one the Truth function gives, by default the map of the
   --  second picture.
   function Run
     (Kind : Scene; Linear : Linear_Part; Shift_U, Shift_V : Real; Noise : Real;
      Miss_U, Miss_V, Sigma : Real; Linear_Miss : Linear_Part := (others => 0.0); Linear_Sigma : Real := Real'Last;
      Columns : Positive := 6; Rows : Positive := 4; Margin : Real := 50.0; Bend : Real := 0.0;
      Light_Gain : Real := 1.0; Light_Offset : Real := 0.0) return Outcome
   is
      First  : constant Pyramid := Pyramid_Of (Picture (Kind, Identity_Part, 0.0, 0.0, Noise));
      Second : constant Pyramid := Pyramid_Of
          (Picture (Kind, Linear, Shift_U, Shift_V, Noise, Bend => Bend, Light_Gain => Light_Gain,
                    Light_Offset => Light_Offset));
      Result : Outcome;
   begin
      Result.Narrowest := Real'Last;
      for J in 0 .. Rows - 1 loop
         for I in 0 .. Columns - 1 loop
            declare
               From : constant Driver.Images.Pixel :=
                 (U => Margin + Real (I) * (Real (Width) - 2.0 * Margin) / Real (Columns) + 0.37,
                  V => Margin + Real (J) * (Real (Height) - 2.0 * Margin) / Real (Rows) + 0.61);
               Truth : constant Driver.Images.Pixel := Moved (Linear, Shift_U, Shift_V, From, Bend);
               Query : constant Prediction :=
                 (From => From, To => (U => Truth.U + Miss_U, V => Truth.V + Miss_V),
                  Linear => (UU => Linear.UU + Linear_Miss.UU, UV => Linear.UV + Linear_Miss.UV,
                             VU => Linear.VU + Linear_Miss.VU, VV => Linear.VV + Linear_Miss.VV),
                  Cov => (UU => Sigma ** 2, UV => 0.0, VV => Sigma ** 2), Linear_Sigma => Linear_Sigma);
               A : constant Answer := Align (First, Second, Query);
            begin
               Result.Asked := Result.Asked + 1;
               if A.Verdict = Found then
                  declare
                     Eu : constant Real := A.To.U - Truth.U;
                     Ev : constant Real := A.To.V - Truth.V;
                     Distance : constant Real := Sqrt (Eu ** 2 + Ev ** 2);
                     Su : constant Real := Sigma_Of (A.Cov.UU);
                     Sv : constant Real := Sigma_Of (A.Cov.VV);
                     Gate : constant Real := Gate_Of (A.Degrees_Of_Freedom);
                  begin
                     Result.Found := Result.Found + 1;
                     Result.Worst := Real'Max (Result.Worst, Distance);
                     Result.Worst_Linear := Real'Max
                       (Result.Worst_Linear,
                        Real'Max (Real'Max (abs (A.Linear.UU - Linear.UU), abs (A.Linear.UV - Linear.UV)),
                                  Real'Max (abs (A.Linear.VU - Linear.VU), abs (A.Linear.VV - Linear.VV))));
                     Result.Axes := Result.Axes + 2;
                     if abs Eu <= Gate * Su then
                        Result.Within_Gate := Result.Within_Gate + 1;
                     end if;
                     if abs Ev <= Gate * Sv then
                        Result.Within_Gate := Result.Within_Gate + 1;
                     end if;
                     Result.Narrowest := Real'Min (Result.Narrowest, Real'Min (Su, Sv));
                     Result.Widest := Real'Max (Result.Widest, Real'Max (Su, Sv));
                     if Distance > 1.0 then
                        Result.Wrong := Result.Wrong + 1;
                     end if;
                  end;
               elsif Result.First_Reason = Matched then
                  Result.First_Reason := A.Because;
               end if;
            end;
         end loop;
      end loop;
      return Result;
   end Run;

   function Describe (O : Outcome) return String is
     ("found" & Natural'Image (O.Found) & " of" & Natural'Image (O.Asked) & ", worst error " & Real'Image (O.Worst)
      & " px, wrong beyond a pixel" & Natural'Image (O.Wrong) & ", first refusal "
      & Reason'Image (O.First_Reason));

   ---------------------------------------------------------------------------

   procedure Translation is
      --  Two renders of one texture, the second shifted by a few pixels, the prediction a pixel and a bit off
      --  and uncertain by one and a half: every point is found, to a few hundredths of a pixel.
      O : Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 11);
      O := Run (Textured, Identity_Part, 3.3, -2.1, 0.0, 1.2, -0.9, 1.5);
      Check (O.Found = O.Asked, "points of a shifted texture were not all found: " & Describe (O));
      Check (O.Worst < 0.05, "a point of a shifted texture was found" & Real'Image (O.Worst) & " px from where it is");
   end Translation;

   procedure Still is
      --  Two identical renders, the prediction a pixel and a half off: the place is where it was, to a few
      --  thousandths of a pixel. A cost read between the pixels with a corner at every pixel centre has its
      --  optimum on a corner, the fit bounces across it and is never called settled.
      O : Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 12);
      O := Run (Textured, Identity_Part, 0.0, 0.0, 0.0, 1.0, -1.2, 1.5);
      Check (O.Found = O.Asked, "identical pictures were not all found: " & Describe (O));
      Check (O.Worst < 0.01, "a point of identical pictures was found" & Real'Image (O.Worst) & " px from where it is");
   end Still;

   procedure Affine is
      --  The second picture turns and stretches the first; the prediction knows the warp only to four hundredths
      --  of each entry. The place and the warp are recovered.
      Warp : constant Linear_Part := (UU => 1.25, UV => 0.10, VU => -0.10, VV => 0.90);
      O : Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 13);
      O := Run (Textured, Warp, -4.0, 2.5, 0.0, 0.8, -0.6, 1.0, Linear_Miss => (0.04, -0.04, 0.04, -0.04),
                Linear_Sigma => 0.08);
      Check (O.Found = O.Asked, "points of a turned and stretched texture were not all found: " & Describe (O));
      Check (O.Worst < 0.1, "a point of a turned texture was found" & Real'Image (O.Worst) & " px from where it is");
      Check (O.Worst_Linear < 0.02, "the linear part was recovered no closer than" & Real'Image (O.Worst_Linear));
   end Affine;

   procedure Scaled_Views is
      --  The second picture shows the scene at less than half the size of the first, and at more than twice it: the
      --  pictures are compared at levels that differ, and the points are found where they are, as when the scale is
      --  one.
      Smaller : constant Linear_Part := (UU => 0.42, UV => 0.0, VU => 0.0, VV => 0.42);
      Larger  : constant Linear_Part := (UU => 2.2, UV => 0.0, VU => 0.0, VV => 2.2);
      O       : Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 26);
      O := Run (Textured, Smaller, 40.0, 30.0, 0.5, 0.4, -0.3, 1.0, Linear_Sigma => 0.1);
      Check (O.Found * 10 >= O.Asked * 8, "a scene seen at 0.42 was found at only" & Natural'Image (O.Found) & " of"
             & Natural'Image (O.Asked) & " points: " & Describe (O));
      Check (O.Worst < 0.3, "a point of a scene seen at 0.42 was found" & Real'Image (O.Worst) & " px from the truth");
      O := Run (Textured, Larger, -140.0, -90.0, 0.5, 0.4, -0.3, 1.0, Linear_Sigma => 0.1, Columns => 3, Rows => 2,
                Margin => 90.0);
      Check (O.Found * 10 >= O.Asked * 8, "a scene seen at 2.2 was found at only" & Natural'Image (O.Found) & " of"
             & Natural'Image (O.Asked) & " points: " & Describe (O));
      Check (O.Worst < 0.3, "a point of a scene seen at 2.2 was found" & Real'Image (O.Worst) & " px from the truth");
   end Scaled_Views;

   procedure Grained is
      --  A grain about three pixels in period, the second picture shifted by under a period, the prediction two
      --  pixels off, so the window holds a period or two and the true basin is narrow (a period is three pixels)
      --  and lies between the pixels: the place is found in the right period.
      O : Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 14);
      O := Run (Grain, Identity_Part, 0.7, 0.3, 0.0, 1.8, -1.5, 1.5, Linear_Sigma => 0.02);
      Check (O.Wrong = 0, "a fine grain gave" & Natural'Image (O.Wrong)
             & " answers more than a pixel from the truth: " & Describe (O));
      Check (O.Found * 10 >= O.Asked * 8,
             "a fine grain was found at only" & Natural'Image (O.Found) & " of" & Natural'Image (O.Asked)
             & " points: " & Describe (O));
   end Grained;

   procedure Faint is
      --  The second picture's texture is a few thousandths of the first's, under the pixels' own noise (half a
      --  level, and the rounding to whole levels): the gradients the second picture has are the noise's, which
      --  are far steeper than the texture's, and an information counted from them is far too high. Whatever the
      --  aligner answers is within its own gate at nearly all axes (a Gaussian would have all but three in a
      --  thousand); most of it is refused.
      O : Outcome;
      Gain : constant Real := 0.003;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 18);
      O := Run (Textured, Identity_Part, 2.6, -1.4, 0.5, -0.7, 0.4, 1.5, Light_Gain => Gain,
                Light_Offset => 128.0 * (1.0 - Gain));
      Check (O.Axes = 0 or else O.Within_Gate * 100 >= O.Axes * 95,
             "only" & Natural'Image (O.Within_Gate) & " of" & Natural'Image (O.Axes)
             & " axes of a faint texture were within their own gates: " & Describe (O));
   end Faint;

   procedure Noisy is
      --  A texture of the usual contrast under noise of eight levels: the noise's gradients are a fifth of the
      --  texture's, and counted as its own they would make the answers a tenth too sure. The errors are within
      --  their own gates at nearly all axes, and most points are found.
      O : Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 19);
      O := Run (Textured, Identity_Part, 2.6, -1.4, 8.0, -0.7, 0.4, 1.5, Columns => 12, Rows => 8);
      Check (O.Found * 10 >= O.Asked * 8,
             "a noisy texture was found at" & Natural'Image (O.Found) & " of" & Natural'Image (O.Asked)
             & " points: " & Describe (O));
      Check (O.Axes > 0 and then O.Within_Gate * 100 >= O.Axes * 95,
             "only" & Natural'Image (O.Within_Gate) & " of" & Natural'Image (O.Axes)
             & " axes of a noisy texture were within their own gates");
   end Noisy;

   procedure Lit is
      --  The second picture is brighter as a whole (a fifth more light and twenty-five levels over), as when
      --  the exposure or the light moved: the fit carries a gain and an offset of the light, so that the place
      --  is found as near as it is when the light has not changed.
      O : Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 17);
      O := Run (Textured, Identity_Part, 2.6, -1.4, 0.8, -0.7, 0.4, 1.2, Light_Gain => 1.2, Light_Offset => 25.0);
      Check (O.Found = O.Asked, "points of a brighter texture were not all found: " & Describe (O));
      Check (O.Worst < 0.1, "a point of a brighter texture was found" & Real'Image (O.Worst) & " px from where it is");
   end Lit;

   procedure Bent is
      --  The second picture shows the scene through a map bent by a wave (two pixels in amplitude, eighty in
      --  wavelength) that no affine map holds: the place a patch gives is the mean displacement over it, off the
      --  displacement at its point by the wave's curvature over the patch. That error is a fifth of a pixel, far
      --  over what the pixels' noise says, and it shrinks with the patch. The answers say how far the patches
      --  pin them (the fit repeated from the answer on the middle of the patch and at coarser levels), so that
      --  the errors are within their own gates at nearly all axes.
      O : Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 16);
      O := Run (Textured, Identity_Part, 1.3, -0.7, 0.5, 0.3, -0.4, 1.5, Linear_Sigma => 0.3, Columns => 12,
                Rows => 8, Bend => 2.0);
      Check (O.Found * 10 >= O.Asked * 8,
             "a bent texture was found at" & Natural'Image (O.Found) & " of" & Natural'Image (O.Asked)
             & " points: " & Describe (O));
      Check (O.Axes > 0 and then O.Within_Gate * 100 >= O.Axes * 95,
             "only" & Natural'Image (O.Within_Gate) & " of" & Natural'Image (O.Axes)
             & " axes of a bent texture were within their own gates");
   end Bent;

   procedure Contour is
      --  Part of the patch moves with a surface in front of the rest (to the right of a line the second picture
      --  shows the scene shifted by half a pixel, to its left by three): the place a patch across the line gives
      --  is neither surface's, and it must not be given as if it were. Each answer is within its own gate of the
      --  place of the surface it is nearer.
      First  : constant Pyramid := Pyramid_Of (Picture (Textured, Identity_Part, 0.0, 0.0, 0.0));
      Second : constant Pyramid :=
        Pyramid_Of (Picture (Textured, Identity_Part, 3.0, 0.0, 0.0, Split => 160.0, Right_Shift => 0.5));
      Axes, Within : Natural := 0;
      Answers : Natural := 0;
   begin
      for K in 0 .. 12 loop
         declare
            From : constant Driver.Images.Pixel := (U => 142.5 + 3.0 * Real (K), V => 100.5);
            Left_Place  : constant Driver.Images.Pixel := (U => From.U + 3.0, V => From.V);
            Right_Place : constant Driver.Images.Pixel := (U => From.U + 0.5, V => From.V);
            --  the surface the point is on: the left one while its place is left of the line
            Truth : constant Driver.Images.Pixel := (if Left_Place.U < 160.0 then Left_Place else Right_Place);
            A : constant Answer :=
              Align (First, Second,
                     (From => From, To => (U => Truth.U + 0.4, V => Truth.V - 0.3), Linear => Identity_Part,
                      Cov => (UU => 1.0, UV => 0.0, VV => 1.0), Linear_Sigma => 0.05));
         begin
            if A.Verdict = Found then
               declare
                  Gate : constant Real := Gate_Of (A.Degrees_Of_Freedom);
               begin
                  Answers := Answers + 1;
                  Axes := Axes + 2;
                  if abs (A.To.U - Truth.U) <= Gate * Sigma_Of (A.Cov.UU) then
                     Within := Within + 1;
                  end if;
                  if abs (A.To.V - Truth.V) <= Gate * Sigma_Of (A.Cov.VV) then
                     Within := Within + 1;
                  end if;
               end;
            end if;
         end;
      end loop;
      Check (Answers > 0, "no point across a contour was found at all");
      Check (Within = Axes, "only" & Natural'Image (Within) & " of" & Natural'Image (Axes)
             & " axes of the points across a contour were within their own gates");
   end Contour;

   procedure Outside is
      --  The prediction lies beyond the second picture, a little and by an absurd distance: not in view, whatever
      --  the pixels are.
      Near   : constant Prediction :=
        (From => (U => 160.5, V => 120.5), To => (U => 400.0, V => 120.0), Cov => (UU => 4.0, UV => 0.0, VV => 4.0),
         others => <>);
      Far    : constant Prediction :=
        (From => (U => 160.5, V => 120.5), To => (U => 1.0E30, V => 120.0), Cov => (UU => 4.0, UV => 0.0, VV => 4.0),
         others => <>);
      First  : constant Pyramid := Pyramid_Of (Picture (Textured, Identity_Part, 0.0, 0.0, 0.0));
      A      : constant Answer := Align (First, First, Near);
      B      : constant Answer := Align (First, First, Far);
   begin
      Check (A.Verdict = Not_In_View, "a prediction beyond the picture was not called out of view: "
             & Verdict'Image (A.Verdict) & " " & Reason'Image (A.Because));
      Check (B.Verdict = Not_In_View, "a prediction an absurd distance beyond the picture was not called out of view: "
             & Verdict'Image (B.Verdict) & " " & Reason'Image (B.Because));
   end Outside;

   procedure Without_Texture is
      --  A flat picture holds nothing to find.
      O : constant Outcome := Run (Flat, Identity_Part, 2.0, 1.0, 0.0, 0.0, 0.0, 1.5);
   begin
      Check (O.Found = 0, "a flat picture gave" & Natural'Image (O.Found) & " answers");
      Check (O.First_Reason = No_Texture,
             "a flat picture was refused for another reason: " & Reason'Image (O.First_Reason));
   end Without_Texture;

   procedure Repeats is
      --  A pattern that repeats every five pixels, with a window wider than five: the places a period apart
      --  explain the patch alike, and none may be called found.
      O : constant Outcome := Run (Periodic, Identity_Part, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0);
   begin
      Check (O.Wrong = 0,
             "a repeating pattern gave" & Natural'Image (O.Wrong) & " answers a period off the truth: " & Describe (O));
      Check (O.Found = 0,
             "a repeating pattern was found at" & Natural'Image (O.Found) & " of" & Natural'Image (O.Asked)
             & " points");
   end Repeats;

   procedure Aperture is
      --  Along an edge only the place across it is known: the answer is near the truth across, as wide as the
      --  picture along, or refused.
      First  : constant Pyramid := Pyramid_Of (Picture (Edge, Identity_Part, 0.0, 0.0, 0.0));
      Second : constant Pyramid := Pyramid_Of (Picture (Edge, Identity_Part, 2.5, 0.0, 0.0));
      Query  : constant Prediction :=
        (From => (U => 160.2, V => 120.3), To => (U => 161.5, V => 120.3), Cov => (UU => 4.0, UV => 0.0, VV => 4.0),
         others => <>);
      A : constant Answer := Align (First, Second, Query);
   begin
      if A.Verdict = Found then
         Check (abs (A.To.U - 162.7) < 0.5, "the place across an edge was" & Real'Image (A.To.U) & ", not 162.7");
         Check (A.Cov.VV > 25.0 * A.Cov.UU,
                "along the edge the variance was" & Real'Image (A.Cov.VV) & ", across" & Real'Image (A.Cov.UU));
         Check (A.Cov.VV > 100.0,
                "the place along an edge was claimed to within" & Real'Image (Sqrt (A.Cov.VV)) & " px");
      end if;
   end Aperture;

   procedure Reach is
      --  The widest window the aligner searches is the one Largest_Window says: a prediction whose window lies a tenth
      --  inside it is searched, one a tenth outside it is refused as too wide.
      First   : constant Pyramid := Pyramid_Of (Picture (Textured, Identity_Part, 0.0, 0.0, 0.0));
      Widest  : constant Real := Largest_Window (First, First);
      Gate    : constant Real := Driver.Uncertain.Threshold (Driver.Uncertain.Vector_Gate (2));

      function Query_Of (Radius : Real) return Prediction is
        (From => (U => 160.5, V => 120.5), To => (U => 160.5, V => 120.5),
         Cov => (UU => (Radius / Gate) ** 2, UV => 0.0, VV => (Radius / Gate) ** 2), others => <>);

      Inside  : constant Answer := Align (First, First, Query_Of (0.9 * Widest));
      Outside : constant Answer := Align (First, First, Query_Of (1.1 * Widest));
      Sigma   : constant Real := Largest_Sigma (First, First);
      Round_Most  : constant Answer :=
        Align (First, First, (From => (U => 160.5, V => 120.5), To => (U => 160.5, V => 120.5),
                              Cov => (UU => Sigma ** 2, UV => 0.0, VV => Sigma ** 2), others => <>));
      Round_Wider : constant Answer :=
        Align (First, First, (From => (U => 160.5, V => 120.5), To => (U => 160.5, V => 120.5),
                              Cov => (UU => (1.01 * Sigma) ** 2, UV => 0.0, VV => (1.01 * Sigma) ** 2),
                              others => <>));
   begin
      Check (Round_Most.Because /= Window_Too_Large, "a prediction of the largest sigma (" & Real'Image (Sigma)
             & " px) was refused as too wide");
      Check (Round_Wider.Because = Window_Too_Large,
             "a prediction of a hundredth more than the largest sigma was searched: "
             & Reason'Image (Round_Wider.Because));
      Check (Widest > 0.0, "no window is searched at all: " & Real'Image (Widest));
      Check (Inside.Because /= Window_Too_Large,
             "a window a tenth inside the widest (" & Real'Image (0.9 * Widest) & " px) was refused as too wide");
      Check (Outside.Because = Window_Too_Large,
             "a window a tenth outside the widest (" & Real'Image (1.1 * Widest) & " px) was searched: "
             & Reason'Image (Outside.Because));
   end Reach;

   procedure Window is
      --  A window wider than the pictures' patches carry at any level, and a mirrored warp: said so.
      First : constant Pyramid := Pyramid_Of (Picture (Textured, Identity_Part, 0.0, 0.0, 0.0));
      Wide  : constant Prediction :=
        (From => (U => 160.5, V => 120.5), To => (U => 160.5, V => 120.5),
         Cov => (UU => 10_000.0, UV => 0.0, VV => 10_000.0), others => <>);
      Mirror : constant Prediction :=
        (From => (U => 160.5, V => 120.5), To => (U => 160.5, V => 120.5),
         Linear => (UU => -1.0, UV => 0.0, VU => 0.0, VV => 1.0),
         Cov => (UU => 1.0, UV => 0.0, VV => 1.0), others => <>);
      A : constant Answer := Align (First, First, Wide);
      B : constant Answer := Align (First, First, Mirror);
   begin
      Check (A.Verdict = Not_Found and then A.Because = Window_Too_Large,
             "a window of a hundred pixels was searched: " & Reason'Image (A.Because));
      Check (B.Because = Bad_Warp, "a mirrored warp was compared: " & Reason'Image (B.Because));
   end Window;

   procedure Narrow_Window is
      --  An answer says whether the window it was searched in was one a patch carries at the full size: a prediction
      --  a pixel uncertain is searched there; one a few pixels uncertain is searched at a coarser size, and its answer,
      --  though found, is not one to rest a fit on before the move is better known.
      First : constant Pyramid := Pyramid_Of (Picture (Textured, Identity_Part, 0.0, 0.0, 0.0));

      function Asked (Sigma : Real) return Answer is
        (Align (First, First,
                (From => (U => 160.5, V => 120.5), To => (U => 160.5, V => 120.5),
                 Cov => (UU => Sigma ** 2, UV => 0.0, VV => Sigma ** 2), others => <>)));

      Near : constant Answer := Asked (1.0);
      Far  : constant Answer := Asked (2.5);
   begin
      Check (Near.Verdict = Found and then Near.Narrow,
             "a prediction a pixel uncertain was not searched at the full size: " & Reason'Image (Near.Because)
             & (if Near.Narrow then ", narrow" else ", not narrow"));
      Check (Far.Verdict = Found and then not Far.Narrow,
             "a prediction two and a half pixels uncertain was "
             & (if Far.Verdict = Found then "found, narrow" else "not found: " & Reason'Image (Far.Because)));
      Check (0.0 < Window_Radius (0.0) and then Window_Radius (0.0) < Window_Radius (1.0)
             and then Window_Radius (1.0) < Window_Radius (2.5),
             "the window of a prediction does not widen with its uncertainty, nor hold a place on a grid when it is exact");
   end Narrow_Window;

   ---------------------------------------------------------------------------
   --  The keyframes of one reference (Driver.Alignment.Keyframes)

   --  An arm turns two joints. The view of the plane it looks at moves with the first joint's amount by a shift, by
   --  a stretch of the picture about its corner, and by a part of the shift that grows with the square of the
   --  amount, which a line through the first keyframes does not follow; it moves with the second joint's amount by
   --  a shift alone.
   Per_Unit  : constant Real_Array := [0.5, -0.3];    --  the shift of the view, pixels per unit of the first joint
   Squared   : constant Real_Array := [0.002, 0.001]; --  and pixels per unit squared
   Other_Per_Unit : constant Real_Array := [0.2, 0.4];   --  and per unit of the second joint
   Stretch   : constant Linear_Part := (UU => 0.0006, UV => 0.0003, VU => -0.0004, VV => 0.0005);
   Readings_Noise : constant Real_Array := [0.001, 0.001];   --  of one reading of each joint
   Pixels_Per_Reading : constant Real_Array := [0.583, 0.447];   --  how far a unit of each joint moves the view

   function Linear_At (Change : Real_Array) return Linear_Part is
     (UU => 1.0 + Change (1) * Stretch.UU, UV => Change (1) * Stretch.UV, VU => Change (1) * Stretch.VU,
      VV => 1.0 + Change (1) * Stretch.VV);

   function Shift_At (Change : Real_Array; Axis : Positive) return Real is
     (Change (1) * Per_Unit (Axis) + Change (1) ** 2 * Squared (Axis) + Change (2) * Other_Per_Unit (Axis));

   type Sweep_Outcome is record
      Asked, Found    : Natural := 0;
      Wrong           : Natural := 0;  --  the answers more than a pixel from the truth
      Worst           : Real := 0.0;   --  the largest distance of an answer from the truth
      Inside          : Natural := 0;  --  the points whose truth lay inside the window the prediction stated
      Largest_Move    : Real := 0.0;   --  the farthest a point moved in the keyframe
      Source          : Keyframes.Source := Keyframes.Supplied;
      Window          : Real := 0.0;   --  the radius of the window the prediction stated at its first point
      First_Reason    : Reason := Matched;   --  why the first refused point was refused
      Stated          : Real := 0.0;   --  the stated standard deviation at the first point
   end record;

   --  One keyframe, at that change of the two joints' readings, predicted from the chain and added to it. The picture
   --  may be taken Jitter pixels along u from where the readings put it (the truth is where they put it): a view that
   --  does not follow the readings, as the aligner's noise does not.
   function Keyframe_Of
     (C : in out Keyframes.Chain; Points : Keyframes.Point_Array; Change : Real_Array;
      Shift : Real_Array := Pixels_Per_Reading; Offered : Keyframes.Prediction_Array := Keyframes.No_Offer;
      Jitter : Real := 0.0)
      return Sweep_Outcome
   is
      Gate    : constant Real := Driver.Uncertain.Threshold (Driver.Uncertain.Vector_Gate (2));
      Image   : constant Driver.Images.Image :=
        Picture (Textured, Linear_At (Change), Shift_At (Change, 1) + Jitter, Shift_At (Change, 2), 0.8);
      Given   : Keyframes.Prediction_Array (Points'Range);
      Answers : Keyframes.Answer_Array (Points'Range);
      Agree   : Keyframes.Flag_Array (Points'Range);
      Result  : Sweep_Outcome;
   begin
      Keyframes.Predict (C, Change, Readings_Noise, Shift, Offered, Given, Result.Source);
      Result.Window := Gate * Sqrt (Given (Points'First).Cov.UU + 1.0 / 12.0);
      Result.Stated := Sqrt (Given (Points'First).Cov.UU);
      Keyframes.Add (C, Image, Change, Given, Answers, Agree);
      for K in Points'Range loop
         declare
            Truth : constant Driver.Images.Pixel :=
              Moved (Linear_At (Change), Shift_At (Change, 1), Shift_At (Change, 2), Points (K));
            Off   : constant Real := Sqrt ((Truth.U - Given (K).To.U) ** 2 + (Truth.V - Given (K).To.V) ** 2);
         begin
            Result.Asked := Result.Asked + 1;
            Result.Largest_Move := Real'Max
              (Result.Largest_Move, Sqrt ((Truth.U - Points (K).U) ** 2 + (Truth.V - Points (K).V) ** 2));
            if Answers (K).Verdict = Found then
               declare
                  Error : constant Real :=
                    Sqrt ((Answers (K).To.U - Truth.U) ** 2 + (Answers (K).To.V - Truth.V) ** 2);
               begin
                  Result.Found := Result.Found + 1;
                  Result.Worst := Real'Max (Result.Worst, Error);
                  if Error > 1.0 then
                     Result.Wrong := Result.Wrong + 1;
                  end if;
               end;
            elsif Result.First_Reason = Matched then
               Result.First_Reason := Answers (K).Because;
            end if;
            if Off <= Gate * Sqrt (Given (K).Cov.UU + 1.0 / 12.0) then
               Result.Inside := Result.Inside + 1;
            end if;
         end;
      end loop;
      return Result;
   end Keyframe_Of;

   function Grid_Points return Keyframes.Point_Array is
      Columns : constant := 6;
      Rows    : constant := 4;
      Margin  : constant := 60.0;
      Result  : Keyframes.Point_Array (1 .. Columns * Rows);
   begin
      for J in 0 .. Rows - 1 loop
         for I in 0 .. Columns - 1 loop
            Result (1 + J * Columns + I) :=
              (U => Margin + Real (I) * (Real (Width) - 2.0 * Margin) / Real (Columns) + 0.37,
               V => Margin + Real (J) * (Real (Height) - 2.0 * Margin) / Real (Rows) + 0.61);
         end loop;
      end loop;
      return Result;
   end Grid_Points;

   procedure Sweep is
      --  Four keyframes of a sweep that quadruples (a quarter of a unit to sixteen, the view moving from under a
      --  pixel to a dozen): each is predicted from the ones before it, with the window its uncertainty makes, and
      --  found where it is. The truth lies inside the stated window at nearly all points and every point is found.
      --  A fifth, four times the fourth, is past what the prediction can follow (a fourfold extrapolation multiplies
      --  the noise of the fields it rests on by dozens): its window is wider than the aligner searches, and it is
      --  refused rather than answered wrongly.
      Followed : constant := 4;
      Points   : constant Keyframes.Point_Array := Grid_Points;
      C        : Keyframes.Chain;
      Outcomes : array (1 .. Followed + 1) of Sweep_Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 21);
      Keyframes.Begin_Reference (C, Picture (Textured, Identity_Part, 0.0, 0.0, 0.8), Points);
      for K in Outcomes'Range loop
         declare
            Amount : constant Real := 0.25 * 4.0 ** (K - 1);
         begin
            Outcomes (K) := Keyframe_Of (C, Points, [Amount, 0.0]);
            if K <= Followed then
               Check (Outcomes (K).Found = Outcomes (K).Asked,
                      "keyframe" & Natural'Image (K) & " of a sweep (the view moved up to"
                      & Real'Image (Outcomes (K).Largest_Move) & " px) was found at"
                      & Natural'Image (Outcomes (K).Found) & " of" & Natural'Image (Outcomes (K).Asked) & " points");
               Check (Outcomes (K).Worst < 0.2,
                      "keyframe" & Natural'Image (K) & " of a sweep was found" & Real'Image (Outcomes (K).Worst)
                      & " px from the truth");
               Check (Outcomes (K).Source = (if K = 1 then Keyframes.New_Direction else Keyframes.Extrapolated),
                      "keyframe" & Natural'Image (K) & " of a sweep was predicted as "
                      & Keyframes.Source'Image (Outcomes (K).Source));
            end if;
         end;
      end loop;
      Check (Keyframes.Keyframes (C) = Outcomes'Length, "the chain holds" & Natural'Image (Keyframes.Keyframes (C))
             & " keyframes, not" & Natural'Image (Outcomes'Length));
      for K in Outcomes'Range loop
         Check (Outcomes (K).Inside * 100 >= Outcomes (K).Asked * 95,
                "the truth lay inside the stated window (" & Real'Image (Outcomes (K).Window) & " px) at only"
                & Natural'Image (Outcomes (K).Inside) & " of" & Natural'Image (Outcomes (K).Asked)
                & " points of keyframe" & Natural'Image (K) & " of the sweep");
      end loop;
      Check (Outcomes (Followed + 1).Wrong = 0,
             "the keyframe past what the prediction follows (the view moved up to"
             & Real'Image (Outcomes (Followed + 1).Largest_Move) & " px) gave"
             & Natural'Image (Outcomes (Followed + 1).Wrong) & " answers more than a pixel from the truth, of"
             & Natural'Image (Outcomes (Followed + 1).Found) & " found");
   end Sweep;

   procedure Directions is
      --  A keyframe at the reference's own readings is the reference again (the same pose: the points have not
      --  moved), and one that turns another joint than any earlier keyframe did is a new direction, predicted
      --  from how far a unit of that joint moves the view, and found. A change that moves the view by no amount
      --  known is predicted to have moved it as far as the aligner searches: nothing narrows the window, so a
      --  texture that repeats within it may be refused, but nothing is answered wrongly.
      Points  : constant Keyframes.Point_Array := Grid_Points;
      Reference : constant Driver.Images.Image := Picture (Textured, Identity_Part, 0.0, 0.0, 0.8);
      Widest  : constant Real := Largest_Window (Pyramid_Of (Reference), Pyramid_Of (Reference));
      C, D    : Keyframes.Chain;
      Still   : Sweep_Outcome;
      First   : Sweep_Outcome;
      Other   : Sweep_Outcome;
      Unknown : Sweep_Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 22);
      Keyframes.Begin_Reference (C, Reference, Points);
      Still := Keyframe_Of (C, Points, [0.0, 0.0]);
      First := Keyframe_Of (C, Points, [1.0, 0.0]);
      Other := Keyframe_Of (C, Points, [0.0, 1.0]);
      Keyframes.Begin_Reference (D, Reference, Points);
      Unknown := Keyframe_Of (D, Points, [8.0, 0.0], Shift => [0.0, 0.0]);
      Check (Still.Source = Keyframes.Same_Pose,
             "a keyframe at the reference's readings was predicted as " & Keyframes.Source'Image (Still.Source));
      Check (Still.Found = Still.Asked and then Still.Worst < 0.1,
             "a keyframe at the reference's readings was found at" & Natural'Image (Still.Found) & " of"
             & Natural'Image (Still.Asked) & " points," & Real'Image (Still.Worst) & " px off");
      Check (First.Source = Keyframes.New_Direction,
             "a joint's first keyframe was predicted as " & Keyframes.Source'Image (First.Source));
      Check (Other.Source = Keyframes.New_Direction,
             "the first keyframe of another joint was predicted as " & Keyframes.Source'Image (Other.Source));
      Check (Other.Found = Other.Asked and then Other.Worst < 0.1,
             "the first keyframe of another joint was found at" & Natural'Image (Other.Found) & " of"
             & Natural'Image (Other.Asked) & " points," & Real'Image (Other.Worst) & " px off");
      Check (Unknown.Source = Keyframes.New_Direction,
             "a change of unknown shift was predicted as " & Keyframes.Source'Image (Unknown.Source));
      Check (Unknown.Window <= Widest and then Unknown.Window > 0.9999 * Widest,
             "a change of unknown shift was given a window of" & Real'Image (Unknown.Window) & " px, not the widest ("
             & Real'Image (Widest) & ")");
      Check (Unknown.Wrong = 0,
             "a change of unknown shift (the view moved up to" & Real'Image (Unknown.Largest_Move) & " px) gave"
             & Natural'Image (Unknown.Wrong) & " answers more than a pixel from the truth, of"
             & Natural'Image (Unknown.Found) & " found");
   end Directions;

   procedure Combined is
      --  The two joints turned one after the other, each by amounts that quadruple, and then both at once: to first order
      --  the view of the keyframe that turned both is the sum of the views of the ones that turned each alone (scaled
      --  to the amounts, the sign of the second joint's reversed), so it is predicted so, with a window narrower than
      --  the one a change that nothing earlier resembles is given (as far as the change can have moved the view), and
      --  found where it is.
      Points   : constant Keyframes.Point_Array := Grid_Points;
      C        : Keyframes.Chain;
      Alone    : array (1 .. 8) of Sweep_Outcome;
      Together : Sweep_Outcome;
      Change   : constant Real_Array := [12.0, -6.0];
      Reach    : constant Real := abs Change (1) * Pixels_Per_Reading (1) + abs Change (2) * Pixels_Per_Reading (2);
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 27);
      Keyframes.Begin_Reference (C, Picture (Textured, Identity_Part, 0.0, 0.0, 0.8), Points);
      for K in 1 .. 4 loop
         Alone (K) := Keyframe_Of (C, Points, [0.25 * 4.0 ** (K - 1), 0.0]);
      end loop;
      for K in 1 .. 4 loop
         Alone (4 + K) := Keyframe_Of (C, Points, [0.0, 0.25 * 4.0 ** (K - 1)]);
      end loop;
      Together := Keyframe_Of (C, Points, Change);
      Check ((for all A of Alone => A.Found = A.Asked), "a keyframe that turned one joint alone was not found at every point");
      Check (Together.Source = Keyframes.Extrapolated,
             "a keyframe that turned both joints at once was predicted as " & Keyframes.Source'Image (Together.Source));
      Check (Together.Stated < Reach,
             "a keyframe that turned both joints at once was given a standard deviation of" & Real'Image (Together.Stated)
             & " px, no less than the" & Real'Image (Reach) & " px the change can have moved the view");
      Check (Together.Inside * 100 >= Together.Asked * 95,
             "the truth lay inside the stated window at only" & Natural'Image (Together.Inside) & " of"
             & Natural'Image (Together.Asked) & " points of a keyframe that turned both joints");
      Check (Together.Found = Together.Asked and then Together.Worst < 0.2,
             "the keyframe that turned both joints was found at" & Natural'Image (Together.Found) & " of"
             & Natural'Image (Together.Asked) & " points," & Real'Image (Together.Worst) & " px off");
   end Combined;

   procedure Faint_Steps is
      --  A sweep that begins with keyframes that move the view by less than the aligner's noise, whose pictures are off
      --  the readings by a few hundredths of a pixel: how curved the move is, read from them, is that noise divided by
      --  the tiny distances the extrapolation went, a thousand times any curvature of the later keyframes, and is
      --  none that the keyframes separate from none. The keyframes that move the view by pixels after them are
      --  predicted as the sweep, found, with the truth inside the stated window.
      Points   : constant Keyframes.Point_Array := Grid_Points;
      Jitter   : constant Real := 0.01;
      C        : Keyframes.Chain;
      Outcomes : array (1 .. 6) of Sweep_Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 28);
      Keyframes.Begin_Reference (C, Picture (Textured, Identity_Part, 0.0, 0.0, 0.8), Points);
      for K in Outcomes'Range loop
         Outcomes (K) :=
           Keyframe_Of (C, Points, [0.0156 * 4.0 ** (K - 1), 0.0],
                        Jitter => (if K <= 3 then (if K mod 2 = 1 then Jitter else -Jitter) else 0.0));
      end loop;
      for K in 5 .. Outcomes'Last loop
         Check (Outcomes (K).Source = Keyframes.Extrapolated,
                "keyframe" & Natural'Image (K) & " of a sweep that began under the noise (the view moved up to"
                & Real'Image (Outcomes (K).Largest_Move) & " px) was predicted as "
                & Keyframes.Source'Image (Outcomes (K).Source));
         Check (Outcomes (K).Found = Outcomes (K).Asked,
                "keyframe" & Natural'Image (K) & " of a sweep that began under the noise was found at"
                & Natural'Image (Outcomes (K).Found) & " of" & Natural'Image (Outcomes (K).Asked) & " points");
         Check (Outcomes (K).Inside * 100 >= Outcomes (K).Asked * 95,
                "the truth lay inside the stated window at only" & Natural'Image (Outcomes (K).Inside) & " of"
                & Natural'Image (Outcomes (K).Asked) & " points of keyframe" & Natural'Image (K)
                & " of a sweep that began under the noise");
      end loop;
   end Faint_Steps;

   --  A model that knows where every point goes in the keyframe at that change, to within the given standard deviation.
   function Model_Of (Points : Keyframes.Point_Array; Change : Real_Array; Sigma : Real)
     return Keyframes.Prediction_Array
   is
      Result : Keyframes.Prediction_Array (Points'Range);
   begin
      for K in Points'Range loop
         Result (K) :=
           (From => Points (K),
            To => Moved (Linear_At (Change), Shift_At (Change, 1), Shift_At (Change, 2), Points (K)),
            Linear => Linear_At (Change), Cov => (UU => Sigma ** 2, UV => 0.0, VV => Sigma ** 2),
            Linear_Sigma => Real'Last);
      end loop;
      return Result;
   end Model_Of;

   procedure Offered_Model is
      --  The caller has a model of its own (a fitted arm's). Where the chain's extrapolation cannot follow (the
      --  view moved fifty pixels in a keyframe sixteen times the last), the model's prediction is used and the
      --  points are found where it puts them; where the model states more uncertainty than the chain, the chain's
      --  is used. The one that states the least wins, whichever it is.
      Points   : constant Keyframes.Point_Array := Grid_Points;
      C        : Keyframes.Chain;
      Far, Near : Sweep_Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 24);
      Keyframes.Begin_Reference (C, Picture (Textured, Identity_Part, 0.0, 0.0, 0.8), Points);
      for K in 1 .. 4 loop
         declare
            Followed : constant Sweep_Outcome := Keyframe_Of (C, Points, [0.25 * 4.0 ** (K - 1), 0.0]);
            pragma Unreferenced (Followed);
         begin
            null;
         end;
      end loop;
      Far := Keyframe_Of (C, Points, [64.0, 0.0], Offered => Model_Of (Points, [64.0, 0.0], 0.2));
      Near := Keyframe_Of (C, Points, [20.0, 0.0], Offered => Model_Of (Points, [20.0, 0.0], 8.0));
      Check (Far.Source = Keyframes.Supplied,
             "a keyframe the chain cannot follow, with a model that states 0.2 px, was predicted as "
             & Keyframes.Source'Image (Far.Source) & " (stated " & Real'Image (Far.Stated) & " px)");
      Check (Far.Found = Far.Asked and then Far.Worst < 0.2,
             "a keyframe predicted by the model (the view moved up to" & Real'Image (Far.Largest_Move)
             & " px) was found at" & Natural'Image (Far.Found) & " of" & Natural'Image (Far.Asked) & " points,"
             & Real'Image (Far.Worst) & " px off");
      Check (Near.Source = Keyframes.Extrapolated,
             "a keyframe the chain follows, with a model that states 8 px, was predicted as "
             & Keyframes.Source'Image (Near.Source));
   end Offered_Model;

   procedure Field_Agreement is
      --  The answers of a keyframe, a third of them wrong places (a window that held the texture's likeness and not
      --  the place), the rest on a smooth field that turns and stretches the picture (a hundred pixels at the far
      --  corner) with the aligner's own scatter: the wrong ones, whatever they claim, are the ones that do not
      --  agree with the field, and the right ones do, though a third of the answers are wrong.
      Points  : constant Keyframes.Point_Array := Grid_Points;
      C       : Keyframes.Chain;
      Answers : Keyframes.Answer_Array (Points'Range);
      Agree   : Keyframes.Flag_Array (Points'Range);
      Wrong_Agreeing, Right_Disagreeing : Natural := 0;
      Wrong   : array (Points'Range) of Boolean := [others => False];
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 25);
      Keyframes.Begin_Reference (C, Picture (Textured, Identity_Part, 0.0, 0.0, 0.8), Points);
      for K in Points'Range loop
         declare
            P : Driver.Images.Pixel renames Points (K);
            Truth : constant Driver.Images.Pixel :=
              (U => P.U + 3.0 + 0.30 * P.U - 0.10 * P.V, V => P.V - 2.0 + 0.10 * P.U + 0.20 * P.V);
         begin
            Wrong (K) := K mod 3 = 0;
            Answers (K) :=
              (Verdict => Found, To => (if Wrong (K) then (U => P.U + 40.0 * Gaussian, V => P.V + 40.0 * Gaussian)
                                       else (U => Truth.U + 0.05 * Gaussian, V => Truth.V + 0.05 * Gaussian)),
               Cov => (UU => 0.01, UV => 0.0, VV => 0.01), others => <>);
         end;
      end loop;
      Keyframes.Agreement (C, Answers, Agree);
      for K in Points'Range loop
         if Wrong (K) and then Agree (K) then
            Wrong_Agreeing := Wrong_Agreeing + 1;
         elsif not Wrong (K) and then not Agree (K) then
            Right_Disagreeing := Right_Disagreeing + 1;
         end if;
      end loop;
      Check (Wrong_Agreeing = 0, Natural'Image (Wrong_Agreeing) & " wrong answers agreed with the field of the right ones");
      Check (Right_Disagreeing * 10 <= Points'Length,
             Natural'Image (Right_Disagreeing) & " of the right answers did not agree with their own field");
   end Field_Agreement;

   procedure Wide_Window is
      --  A prediction as uncertain as the aligner still searches, of a warp that shrinks the first picture to the
      --  last of its sizes: the window is a few hundred pixels across at the only size left, and has a candidate at
      --  every pixel of it, more than a task's stack holds. Asked of four tasks at once, as the keyframes' points
      --  are, every point is answered, found or not, and none of the tasks dies.
      type Bytes_Access is access Driver.Bytes.Byte_Array;
      use type Driver.Bytes.Offset;
      Columns : constant := 640;
      Rows    : constant := 480;
      Data    : constant Bytes_Access := new Driver.Bytes.Byte_Array (1 .. 3 * Columns * Rows);
      type Asks is array (1 .. 4) of Boolean;
      Asked   : Asks := [others => False];
   begin
      for Row in 0 .. Rows - 1 loop
         for Column in 0 .. Columns - 1 loop
            declare
               V : constant Real := Brightness (Textured, Real (Column) + 0.5, Real (Row) + 0.5);
               L : constant Driver.Bytes.Byte := Driver.Bytes.Byte (Integer'Max (0, Integer'Min (255, Integer (Real'Rounding (V)))));
               K : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (Row * Columns + Column));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      declare
         First  : constant Pyramid := Pyramid_Of (Driver.Images.Create (Columns, Rows, Data.all));
         Shrink : constant Real := 0.5 ** (Levels (First) - 1);
         Sigma  : constant Real := Largest_Sigma (First, First);

         procedure Piece (Index : Positive) is
            Answer : constant Alignment.Answer :=
              Align (First, First,
                     (From => (U => 300.0 + 20.0 * Real (Index), V => 240.0), To => (U => 320.0, V => 240.0),
                      Linear => (UU => Shrink, UV => 0.0, VU => 0.0, VV => Shrink),
                      Cov => (UU => Sigma ** 2, UV => 0.0, VV => Sigma ** 2), Linear_Sigma => Real'Last));
            pragma Unreferenced (Answer);
         begin
            Asked (Index) := True;
         end Piece;

         procedure Run is new Driver.Parallel_For (Piece);
      begin
         Run (1, 4);
         Check (Asked = Asks'[others => True], "a window of a few hundred pixels at the finest size left was not searched by every task");
      end;
   end Wide_Window;

   procedure Settled is
      --  A prediction that is right for two thirds of the points and states as much of the rest, which it puts a dozen
      --  pixels from where they are: the window it makes refuses them. The others make a smooth field of the
      --  keyframe's displacements, which says where those are; they are asked again from it and found where they
      --  are, none wrong, and the keyframe's points all agree with the field.
      Points  : constant Keyframes.Point_Array := Grid_Points;
      Change  : constant Real_Array := [6.0, 0.0];
      C       : Keyframes.Chain;
      Given   : Keyframes.Prediction_Array := Model_Of (Points, Change, 0.2);
      Answers : Keyframes.Answer_Array (Points'Range);
      Agree   : Keyframes.Flag_Array (Points'Range);
      Off     : Natural := 0;   --  the points the prediction put away from where they are
      Seen, Wrong, Agreeing : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 26);
      Keyframes.Begin_Reference (C, Picture (Textured, Identity_Part, 0.0, 0.0, 0.8), Points);
      for K in Points'Range loop
         if K mod 3 = 0 then
            Off := Off + 1;
            Given (K).To := (U => Given (K).To.U + 12.0, V => Given (K).To.V - 9.0);
         end if;
      end loop;
      Keyframes.Add (C, Picture (Textured, Linear_At (Change), Shift_At (Change, 1), Shift_At (Change, 2), 0.8),
                     Change, Given, Answers, Agree);
      for K in Points'Range loop
         if Agree (K) then
            Agreeing := Agreeing + 1;
         end if;
         if Answers (K).Verdict = Found then
            Seen := Seen + 1;
            declare
               Truth : constant Driver.Images.Pixel :=
                 Moved (Linear_At (Change), Shift_At (Change, 1), Shift_At (Change, 2), Points (K));
            begin
               if Sqrt ((Answers (K).To.U - Truth.U) ** 2 + (Answers (K).To.V - Truth.V) ** 2) > 1.0 then
                  Wrong := Wrong + 1;
               end if;
            end;
         end if;
      end loop;
      Check (Off > 0 and then Seen = Points'Length,
             "the points a prediction put away from where they are were found at only" & Natural'Image (Seen)
             & " of" & Natural'Image (Points'Length) & " (" & Natural'Image (Off) & " were put away)");
      Check (Wrong = 0, Natural'Image (Wrong) & " answers more than a pixel from the truth after being asked again");
      Check (Agreeing * 10 >= Points'Length * 9,
             "only" & Natural'Image (Agreeing) & " of" & Natural'Image (Points'Length) & " points agree with the field");
   end Settled;

   procedure Contaminated is
      --  An earlier keyframe turned the second joint as well as the first: its points moved by the second joint's
      --  share too, which a multiple of it carries along. The prediction from it states that part as an
      --  uncertainty, and the truth lies inside the window it makes. (The first joint's four units and the second's
      --  two move the view by a pixel more than the first alone would.)
      Points : constant Keyframes.Point_Array := Grid_Points;
      C      : Keyframes.Chain;
      Mixed, Doubled : Sweep_Outcome;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 23);
      Keyframes.Begin_Reference (C, Picture (Textured, Identity_Part, 0.0, 0.0, 0.8), Points);
      Mixed := Keyframe_Of (C, Points, [4.0, 2.0]);
      Doubled := Keyframe_Of (C, Points, [8.0, 0.0]);
      Check (Doubled.Source = Keyframes.Extrapolated,
             "a keyframe along an earlier one that also turned another joint was predicted as "
             & Keyframes.Source'Image (Doubled.Source));
      Check (Doubled.Inside * 100 >= Doubled.Asked * 95,
             "the truth lay inside the stated window at only" & Natural'Image (Doubled.Inside) & " of"
             & Natural'Image (Doubled.Asked) & " points of a keyframe predicted from one that turned another too");
      Check (Mixed.Found = Mixed.Asked, "the mixed keyframe was found at" & Natural'Image (Mixed.Found) & " of"
             & Natural'Image (Mixed.Asked) & " points");
   end Contaminated;

   procedure Register is
   begin
      Driver.Tests.Register
        ("alignment.wide", "a window as wide as the aligner searches, at the finest size left, takes more of a task's stack than it has",
         Wide_Window'Access);
      Driver.Tests.Register
        ("alignment.settled", "the points a prediction put away from where they are, which its window refused, are "
         & "not asked again from the field the others make and found", Settled'Access);
      Driver.Tests.Register
        ("alignment.translation", "a texture shifted by a few pixels is not found where it is", Translation'Access);
      Driver.Tests.Register
        ("alignment.still", "identical pictures are not settled at the place they agree at", Still'Access);
      Driver.Tests.Register
        ("alignment.affine", "the warp of a turned and stretched picture is not recovered", Affine'Access);
      Driver.Tests.Register
        ("alignment.scale", "a scene seen at less than half or more than twice the size is not found where it is",
         Scaled_Views'Access);
      Driver.Tests.Register
        ("alignment.grain", "a grain of a few pixels' period is found a period off", Grained'Access);
      Driver.Tests.Register
        ("alignment.light", "a texture lit more brightly is found away from where it is", Lit'Access);
      Driver.Tests.Register
        ("alignment.faint", "a texture under the noise is answered with a precision that its noise gives",
         Faint'Access);
      Driver.Tests.Register
        ("alignment.noisy", "a noisy texture is answered with a precision that its noise-free gradients give",
         Noisy'Access);
      Driver.Tests.Register
        ("alignment.bent", "a texture bent by a wave is given a place more precise than the wave allows", Bent'Access);
      Driver.Tests.Register
        ("alignment.contour", "a patch across a contour is given a precise place of neither surface", Contour'Access);
      Driver.Tests.Register
        ("alignment.outside", "a prediction beyond the picture is not called out of view", Outside'Access);
      Driver.Tests.Register ("alignment.flat", "a flat picture is called found", Without_Texture'Access);
      Driver.Tests.Register
        ("alignment.repeats", "a pattern that repeats within the window is called found", Repeats'Access);
      Driver.Tests.Register
        ("alignment.aperture", "the place along an edge is claimed as well known as the place across it",
         Aperture'Access);
      Driver.Tests.Register
        ("alignment.reach", "the widest window the aligner searches is not the one it says", Reach'Access);
      Driver.Tests.Register
        ("alignment.window", "a window wider than a patch carries, or a mirror, is compared", Window'Access);
      Driver.Tests.Register
        ("alignment.narrow", "an answer does not say whether its window was one a patch carries", Narrow_Window'Access);
      Driver.Tests.Register
        ("alignment.sweep", "the keyframes of a sweep are not found where the earlier ones put them", Sweep'Access);
      Driver.Tests.Register
        ("alignment.directions", "a keyframe at the reference's readings, or at another joint's, is not found",
         Directions'Access);
      Driver.Tests.Register
        ("alignment.combined", "a keyframe that turned two joints at once is not the sum of the views of each alone",
         Combined'Access);
      Driver.Tests.Register
        ("alignment.faintsteps", "a sweep that begins under the noise is not believed less for it",
         Faint_Steps'Access);
      Driver.Tests.Register
        ("alignment.offered", "a model of the caller's that states less than the chain is not the one used",
         Offered_Model'Access);
      Driver.Tests.Register
        ("alignment.field", "answers a third of which are wrong places are not told from the right ones by their field",
         Field_Agreement'Access);
      Driver.Tests.Register
        ("alignment.contaminated", "a keyframe that also turned another joint is a node with no uncertainty of that",
         Contaminated'Access);
   end Register;

end Driver.Alignment.Tests;
