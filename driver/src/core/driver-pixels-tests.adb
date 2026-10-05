with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Images;
with Driver.Tests;

package body Driver.Pixels.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      U1 : constant Real := 1.0 - Real (Ada.Numerics.Float_Random.Random (Gen));
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   Side : constant := 128;

   --  A grey frame: a smooth ramp, plus Lift inside the square [32, 64), plus
   --  Gaussian noise of the given sigma, rounded to whole levels.
   function Frame (Noise, Lift : Real) return Driver.Images.Image is
      use Driver.Bytes;
      use type Driver.Bytes.Offset;
      Data : Byte_Array (1 .. 3 * Side * Side);
   begin
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            declare
               Inside : constant Boolean := Row in 32 .. 63 and then Column in 32 .. 63;
               V : constant Real := 60.0 + Real (Row + Column) * 0.5 + (if Inside then Lift else 0.0) + Noise * Gaussian;
               L : constant Byte := Byte (Integer'Max (0, Integer'Min (255, Integer (Real'Rounding (V)))));
               K : constant Offset := Offset (3 * (Row * Side + Column));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Side, Side, Data);
   end Frame;

   function View_Of (Frames, Noise, Lift : Real) return View is
      V : View := Empty (Side, Side);
   begin
      for I in 1 .. Natural (Frames) loop
         Add (V, Frame (Noise, Lift));
      end loop;
      return V;
   end View_Of;

   function Changed_Inside (M : Driver.Images.Mask; Inside : Boolean) return Natural is
      N : Natural := 0;
   begin
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            if (Row in 32 .. 63 and then Column in 32 .. 63) = Inside and then Driver.Images.Contains (M, Column, Row) then
               N := N + 1;
            end if;
         end loop;
      end loop;
      return N;
   end Changed_Inside;

   procedure Still_Noise is
      --  Two views of four frames each of a still scene with noise of two
      --  levels: every pixel is tested, as one of a family of as many as the
      --  view has pixels, so a still view passes for changed no oftener than
      --  one test does, about 0.27 % of views, not of pixels.
      Pixels : constant Natural := Side * Side;
      Total  : Natural := 0;
      Rounds : constant := 4;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 31);
      for R in 1 .. Rounds loop
         declare
            C : constant Comparison := Compare (View_Of (4.0, 2.0, 0.0), View_Of (4.0, 2.0, 0.0));
         begin
            Check (C.Trusted, "a still scene was not trusted");
            Check_Close (C.Spread, 2.0 * Sqrt (2.0 / 4.0), 0.05, "the spread of still pixels' differences");
            Total := Total + Driver.Images.Count (C.Changed);
         end;
      end loop;
      Check (Total = 0, "still pixels called changed:" & Natural'Image (Total) & " of" & Natural'Image (Rounds * Pixels));
   end Still_Noise;

   procedure Real_Change is
      C : Comparison;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 37);
      --  Twenty levels against noise of two, fourteen sigmas of the means'
      --  difference: every pixel of the square is seen, none outside it.
      C := Compare (View_Of (4.0, 2.0, 0.0), View_Of (4.0, 2.0, 20.0));
      Check (C.Trusted, "a change of a thirty-second of the view was not trusted");
      Check (Changed_Inside (C.Changed, True) = 32 * 32, "a change of fourteen sigmas was missed:"
             & Natural'Image (Changed_Inside (C.Changed, True)) & " of 1024");
      Check (Changed_Inside (C.Changed, False) = 0, "a change of the square spilled outside it:"
             & Natural'Image (Changed_Inside (C.Changed, False)));
   end Real_Change;

   procedure Noiseless_Renders is
      --  A renderer that repeats itself exactly: nothing in a pixel varies
      --  but the floor, the quantization of two means, and a change has to
      --  stand out of that by the family's multiple. Two frames each: the
      --  floor is 0.29 of a level, the multiple about 5.2, so 1.5 levels.
      C : Comparison;
   begin
      C := Compare (View_Of (2.0, 0.0, 0.0), View_Of (2.0, 0.0, 0.0));
      Check (C.Trusted and then Driver.Images.Count (C.Changed) = 0, "identical renders called changed");
      Check_Close (C.Spread, Sqrt (2.0 / 12.0 / 2.0), 1.0e-9, "the floor of two frames each");
      C := Compare (View_Of (2.0, 0.0, 0.0), View_Of (2.0, 0.0, 1.0));
      Check (Changed_Inside (C.Changed, True) = 0, "a one-level change of an exact render stood out of the quantization");
      C := Compare (View_Of (2.0, 0.0, 0.0), View_Of (2.0, 0.0, 2.0));
      Check (Changed_Inside (C.Changed, True) = 32 * 32 and then Changed_Inside (C.Changed, False) = 0,
             "a two-level change of an exact render was not seen exactly:" & Natural'Image (Changed_Inside (C.Changed, True))
             & Natural'Image (Changed_Inside (C.Changed, False)));
      --  One frame each: the floor is 0.41 of a level, so two levels do not
      --  stand out and three do.
      C := Compare (View_Of (1.0, 0.0, 0.0), View_Of (1.0, 0.0, 2.0));
      Check (Changed_Inside (C.Changed, True) = 0, "one frame each claimed a two-level change");
      C := Compare (View_Of (1.0, 0.0, 0.0), View_Of (1.0, 0.0, 3.0));
      Check (Changed_Inside (C.Changed, True) = 32 * 32, "one frame each missed a three-level change");
   end Noiseless_Renders;

   --  A renderer's lighting moves with whatever moves in it: a frame lit by
   --  Light levels more at one side than the other, differently at every pixel
   --  by a level either way, with the square lifted by Lift as before and no
   --  noise of its own. Light 0 is the scene unlit.
   function Lit_Frame (Light, Lift : Real) return Driver.Images.Image is
      use Driver.Bytes;
      use type Driver.Bytes.Offset;
      Data : Byte_Array (1 .. 3 * Side * Side);
   begin
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            declare
               Inside : constant Boolean := Row in 32 .. 63 and then Column in 32 .. 63;
               Moved  : constant Real :=
                 Light * (1.0 + 2.0 * Real (Column) / Real (Side - 1) + Real ((Row * 7 + Column * 13) mod 3) - 1.0);
               V : constant Real := 60.0 + Real (Row + Column) * 0.5 + (if Inside then Lift else 0.0) + Moved;
               L : constant Byte := Byte (Integer'Max (0, Integer'Min (255, Integer (Real'Rounding (V)))));
               K : constant Offset := Offset (3 * (Row * Side + Column));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Side, Side, Data);
   end Lit_Frame;

   function Lit_View (Frames : Positive; Light, Lift : Real) return View is
      V : View := Empty (Side, Side);
   begin
      for I in 1 .. Frames loop
         Add (V, Lit_Frame (Light, Lift));
      end loop;
      return V;
   end Lit_View;

   procedure Lighting_Is_Not_Change is
      --  Two exact renders of one scene, the second lit one to three levels
      --  more, differently at every pixel, with a square lifted by forty
      --  levels. The lighting is not a change of anything; the square is.
      C : constant Comparison := Compare (Lit_View (2, 0.0, 0.0), Lit_View (2, 1.0, 40.0));
   begin
      Check (C.Trusted, "a view with a lifted square and moved lighting was not trusted");
      Check (Changed_Inside (C.Changed, True) = 32 * 32, "the lifted square was not seen whole:"
             & Natural'Image (Changed_Inside (C.Changed, True)) & " of 1024");
      Check (Changed_Inside (C.Changed, False) = 0, "lighting of one to three levels was called a change at"
             & Natural'Image (Changed_Inside (C.Changed, False)) & " of" & Natural'Image (Side * Side - 32 * 32) & " pixels");
      Check (C.Spread > 0.5 and then C.Spread < 1.5, "the lighting's spread was measured as" & Real'Image (C.Spread)
             & " levels, not about one");
   end Lighting_Is_Not_Change;

   --  A frame whose first Up rows are lifted by Lift, the Down rows after them
   --  lowered by it, with Gaussian noise of the given sigma.
   function Banded_Frame (Noise : Real; Up, Down : Natural; Lift : Real) return Driver.Images.Image is
      use Driver.Bytes;
      use type Driver.Bytes.Offset;
      Data : Byte_Array (1 .. 3 * Side * Side);
   begin
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            declare
               Moved : constant Real := (if Row < Up then Lift elsif Row < Up + Down then -Lift else 0.0);
               V : constant Real := 100.0 + Real (Column) * 0.5 + Moved + Noise * Gaussian;
               L : constant Byte := Byte (Integer'Max (0, Integer'Min (255, Integer (Real'Rounding (V)))));
               K : constant Offset := Offset (3 * (Row * Side + Column));
            begin
               Data (K + 1) := L;
               Data (K + 2) := L;
               Data (K + 3) := L;
            end;
         end loop;
      end loop;
      return Driver.Images.Create (Side, Side, Data);
   end Banded_Frame;

   function Banded_View (Frames : Positive; Noise : Real; Up, Down : Natural; Lift : Real) return View is
      V : View := Empty (Side, Side);
   begin
      for I in 1 .. Frames loop
         Add (V, Banded_Frame (Noise, Up, Down, Lift));
      end loop;
      return V;
   end Banded_View;

   procedure Most_Of_The_View is
      --  The measurement holds while the pixels that did not change are the
      --  majority. Just under half the view lifted: answered, and right. Half
      --  or more called changed: not trusted, and no pixel marked.
      Still  : constant View := Banded_View (4, 2.0, 0, 0, 0.0);
      Third  : constant Natural := Side * 3 / 8;
      C      : Comparison;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 41);
      --  Three eighths of the rows lifted by forty levels against noise of two.
      C := Compare (Still, Banded_View (4, 2.0, Third, 0, 40.0));
      Check (C.Trusted, "a view three eighths changed was not trusted");
      Check (Driver.Images.Count (C.Changed) = Third * Side, "three eighths of the view changed, and"
             & Natural'Image (Driver.Images.Count (C.Changed)) & " pixels were marked, not" & Natural'Image (Third * Side));
      --  Forty-five in a hundred of the rows (58 of 128), one way only: the
      --  median is pulled into the noise of the unchanged pixels, and the
      --  spread still comes out of them.
      C := Compare (Still, Banded_View (4, 2.0, 58, 0, 40.0));
      Check (C.Trusted and then Driver.Images.Count (C.Changed) = 58 * Side,
             "a view 45 % changed in one direction was not answered right:" & Natural'Image (Driver.Images.Count (C.Changed)));
      --  Three tenths up, a quarter down, 45 % unchanged: the unchanged are
      --  the largest group, not the majority, and more than half the view is
      --  beyond them.
      C := Compare (Still, Banded_View (4, 2.0, Side * 3 / 10, Side / 4, 40.0));
      Check (not C.Trusted and then Driver.Images.Count (C.Changed) = 0,
             "a view whose unchanged part was 45 % of it was trusted or marked pixels");
      --  Two fifths up, three tenths down, three tenths unchanged.
      C := Compare (Still, Banded_View (4, 2.0, Side * 2 / 5, Side * 3 / 10, 40.0));
      Check (not C.Trusted and then Driver.Images.Count (C.Changed) = 0,
             "a view whose unchanged part was 30 % of it was trusted or marked pixels");
      --  The same with exact renders, where every unchanged pixel's difference
      --  is one and the same number: the pixels at the median distance are
      --  most of the view, and the nearer half is the ones below it.
      declare
         Exact : constant View := Banded_View (2, 0.0, 0, 0, 0.0);
      begin
         C := Compare (Exact, Banded_View (2, 0.0, Side * 2 / 5, Side * 3 / 10, 40.0));
         Check (not C.Trusted and then Driver.Images.Count (C.Changed) = 0,
                "an exact render of which 30 % was unchanged was trusted or marked pixels");
         C := Compare (Exact, Banded_View (2, 0.0, 58, 0, 40.0));
         Check (C.Trusted and then Driver.Images.Count (C.Changed) = 58 * Side,
                "an exact render 45 % changed was not answered right:" & Natural'Image (Driver.Images.Count (C.Changed)));
      end;
   end Most_Of_The_View;

   procedure Bulk_Reads is
      V : constant View := View_Of (4.0, 2.0, 0.0);
      M : Real_Array (1 .. Side * Side);
      S : Real_Array (1 .. Side * Side);
   begin
      Means (V, M);
      Variances (V, S);
      for Row in 0 .. Side - 1 loop
         for Column in 0 .. Side - 1 loop
            if M (Row * Side + Column + 1) /= Mean (V, Column, Row)
              or else S (Row * Side + Column + 1) /= Variance (V, Column, Row)
            then
               Check (False, "the whole-view reads differ at column" & Column'Image & ", row" & Row'Image);
               return;
            end if;
         end loop;
      end loop;
   end Bulk_Reads;

   procedure Empty_In_A_Task is
      --  The decider runs the estimators in a task with the default stack, so
      --  an empty view of a VGA frame must be made there.
      protected Result is
         procedure Set (Made : Boolean);
         function Get return Boolean;
      private
         Value : Boolean := False;
      end Result;

      protected body Result is
         procedure Set (Made : Boolean) is
         begin
            Value := Made;
         end Set;

         function Get return Boolean is (Value);
      end Result;
   begin
      declare
         task Maker;

         task body Maker is
            V : constant View := Empty (640, 480);
         begin
            Result.Set (Width (V) = 640 and then Height (V) = 480);
         exception
            when others =>
               Result.Set (False);
         end Maker;
      begin
         null;   --  the block waits for Maker to finish
      end;
      Check (Result.Get, "an empty VGA view could not be made in a task with the default stack");
   end Empty_In_A_Task;

   procedure Register is
   begin
      Driver.Tests.Register ("pixels.still", "still pixels are called changed more often than Z promises",
                             Still_Noise'Access);
      Driver.Tests.Register ("pixels.change", "a change far beyond the noise is missed or spills", Real_Change'Access);
      Driver.Tests.Register ("pixels.renders", "exact renders are judged without the quantization floor",
                             Noiseless_Renders'Access);
      Driver.Tests.Register ("pixels.lighting", "a renderer's lighting moving by a few levels is called a change",
                             Lighting_Is_Not_Change'Access);
      Driver.Tests.Register ("pixels.majority", "a view mostly changed is answered as if it were not",
                             Most_Of_The_View'Access);
      Driver.Tests.Register ("pixels.bulk", "the whole-view means or variances differ from the per-pixel ones",
                             Bulk_Reads'Access);
      Driver.Tests.Register ("pixels.task", "an empty VGA view overflows a task's default stack", Empty_In_A_Task'Access);
   end Register;

end Driver.Pixels.Tests;
