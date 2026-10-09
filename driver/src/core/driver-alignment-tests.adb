with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Images;
with Driver.Tests;

package body Driver.Alignment.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;

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

   procedure Register is
   begin
      Driver.Tests.Register
        ("alignment.translation", "a texture shifted by a few pixels is not found where it is", Translation'Access);
      Driver.Tests.Register
        ("alignment.still", "identical pictures are not settled at the place they agree at", Still'Access);
      Driver.Tests.Register
        ("alignment.affine", "the warp of a turned and stretched picture is not recovered", Affine'Access);
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
        ("alignment.window", "a window wider than a patch carries, or a mirror, is compared", Window'Access);
   end Register;

end Driver.Alignment.Tests;
