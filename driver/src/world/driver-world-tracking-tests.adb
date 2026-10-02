with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Bytes;
with Driver.Conventions;
with Driver.Distributions;
with Driver.Tests;

package body Driver.World.Tracking.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Images;
   use Driver.Tests;
   use type Driver.Bytes.Offset;
   use type Driver.Clock.Beat;

   Gen : Ada.Numerics.Float_Random.Generator;

   function Gaussian return Real is
      --  The generator returns [0, 1] with 1 included: U1 is drawn on (0, 1].
      U1 : Real;
      U2 : constant Real := Real (Ada.Numerics.Float_Random.Random (Gen));
   begin
      loop
         U1 := Real (Ada.Numerics.Float_Random.Random (Gen));
         exit when U1 > 0.0;
      end loop;
      return Sqrt (-2.0 * Log (U1)) * Cos (2.0 * Ada.Numerics.Pi * U2);
   end Gaussian;

   W : constant := 64;
   H : constant := 48;

   --  A textured table with a bright square on it, its corner at (X, Y); each
   --  frame carries a grey level of sensor noise.
   function Frame (X, Y : Natural) return Image is
      Data : Driver.Bytes.Byte_Array (1 .. 3 * W * H);
   begin
      for R in 0 .. H - 1 loop
         for C in 0 .. W - 1 loop
            declare
               Base  : constant Real :=
                 (if C in X .. X + 11 and then R in Y .. Y + 11 then 220.0
                  else 100.0 + 40.0 * Sin (Real (C) * 0.5) * Cos (Real (R) * 0.3));
               Level : constant Integer := Integer (Real'Rounding (Base + Gaussian));
               K     : constant Driver.Bytes.Offset := Driver.Bytes.Offset (3 * (R * W + C));
            begin
               for I in 1 .. 3 loop
                  Data (K + Driver.Bytes.Offset (I)) := Driver.Bytes.Byte (Integer'Max (0, Integer'Min (255, Level)));
               end loop;
            end;
         end loop;
      end loop;
      return Create (W, H, Data);
   end Frame;

   function Square (X, Y : Natural) return Mask is
      M : Mask := Create (W, H);
   begin
      for R in Y .. Y + 11 loop
         for C in X .. X + 11 loop
            Include (M, C, R);
         end loop;
      end loop;
      return M;
   end Square;

   function Same (A, B : Mask) return Boolean is
     (Width (A) = Width (B) and then Height (A) = Height (B)
      and then (for all R in 0 .. Height (A) - 1 => (for all C in 0 .. Width (A) - 1 =>
                  Contains (A, C, R) = Contains (B, C, R))));

   procedure Noise_Keeps_It is
      --  Four hundred noisy still beats: the region is lost no more often
      --  than its one test per beat lets pass by chance.
      Beats    : constant := 400;
      Rate     : constant Real := Driver.Distributions.Gaussian_Two_Sided_Tail (Driver.Conventions.Z);
      T        : Track;
      Losses   : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 41);
      T := Start (Square (20, 15), Frame (20, 15), 0);
      for B in 1 .. Beats loop
         Observe (T, Frame (20, 15), Driver.Clock.Beat (B), Still => True);
         if State (T) = Lost then
            Losses := Losses + 1;
            T := Start (Square (20, 15), Frame (20, 15), Driver.Clock.Beat (B));
         end if;
      end loop;
      Check (Real (Losses) <= Rate * Real (Beats) + Driver.Conventions.Z * Sqrt (Rate * Real (Beats)),
             "a still thing was lost" & Losses'Image & " times in" & Beats'Image & " beats");
      Check (Seen (T), "a still thing is not seen");
      Observe (T, Frame (20, 15), Beats + 1, Still => False);
      Check (not Seen (T), "a thing is seen while the body moves");
   end Noise_Keeps_It;

   procedure Moved_And_Found is
      T : Track;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 43);
      T := Start (Square (20, 15), Frame (20, 15), 0);
      for B in 1 .. 3 loop
         Observe (T, Frame (20, 15), Driver.Clock.Beat (B), Still => True);
      end loop;
      Check (State (T) = Holding, "a still thing was lost");
      --  Pushed ten pixels right; seen while it moves, then still again.
      Observe (T, Frame (25, 15), 4, Still => False);
      Observe (T, Frame (30, 15), 5, Still => True);
      Check (State (T) = Lost, "a thing that moved was not lost");
      --  It looks again with two frames taken after the change.
      Observe (T, Frame (30, 15), 6, Still => True);
      Check (Wants_Match (T), "a lost thing is not looked for at a still beat");
      declare
         Points  : constant Driver.Instrument.Point_Array := Match_Points (T);
         Answers : Driver.Instrument.Answer_Array (Points'Range);
      begin
         --  The square's edge of 44 pixels whole, a sample of its 100 inside
         --  pixels, and a sample of the box around it.
         Check (Region_Points (T) >= 44 and then Region_Points (T) < 144 and then Points'Length > Region_Points (T),
                "the region's edge is not matched whole, its inside not by a sample, or nothing around it");
         for K in Points'Range loop
            if K <= Region_Points (T) then
               --  Its pixels went ten to the right and came back, give or
               --  take a fifth of a pixel; one in ten does not come back.
               Answers (K) := (Found     => True,
                               To        => (U => Points (K).U + 10.0, V => Points (K).V),
                               Back      => (U => Points (K).U + 0.2 * Gaussian + (if K mod 10 = 0 then 6.0 else 0.0),
                                             V => Points (K).V + 0.2 * Gaussian),
                               Certainty => 1.0);
            else
               --  The table around it stayed where it was.
               Answers (K) := (Found     => True,
                               To        => (U => Points (K).U + 0.2 * Gaussian, V => Points (K).V + 0.2 * Gaussian),
                               Back      => (U => Points (K).U + 0.2 * Gaussian, V => Points (K).V + 0.2 * Gaussian),
                               Certainty => 1.0);
            end if;
         end loop;
         Asked_Match (T);
         Matched (T, Points, Answers);
      end;
      Check (Wants_Segment (T), "pixels that came back did not ask for the thing to be segmented");
      declare
         Around : Driver.Instrument.Box;
         At_Point : Driver.Instrument.Pixel;
      begin
         Segment_Prompt (T, Around, At_Point);
         Check (Around.X0 >= 30.0 and then Around.X1 <= 42.0 and then At_Point.U in 30.0 .. 42.0,
                "the prompt is not where the pixels went");
      end;
      Asked_Segment (T);
      Segmented (T, Square (30, 15));
      Check (State (T) = Holding and then Same (Region (T), Square (30, 15)) and then Measured_At (T) = 6,
             "the thing found again is not held at its new place");
      Observe (T, Frame (30, 15), 7, Still => True);
      Observe (T, Frame (30, 15), 8, Still => True);
      Check (Seen (T), "the thing found again is not seen");
   end Moved_And_Found;

   procedure Nothing_Comes_Back is
      T : Track;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 47);
      T := Start (Square (20, 15), Frame (20, 15), 0);
      Observe (T, Frame (20, 15), 1, Still => True);
      Observe (T, Frame (20, 15), 2, Still => True);
      Observe (T, Frame (45, 30), 3, Still => True);
      Observe (T, Frame (45, 30), 4, Still => True);
      Check (Wants_Match (T), "a thing that left was not looked for");
      declare
         Points  : constant Driver.Instrument.Point_Array := Match_Points (T);
         Answers : Driver.Instrument.Answer_Array (Points'Range);
      begin
         for K in Points'Range loop
            if K <= Region_Points (T) then
               --  Three in four go nowhere; the rest come back from far off.
               Answers (K) := (Found => K mod 4 = 0, To => (U => 50.0, V => 40.0),
                               Back => (U => Points (K).U + 15.0 + Real (K mod 7), V => Points (K).V - 9.0),
                               Certainty => 0.1);
            else
               Answers (K) := (Found => True, To => Points (K),
                               Back => (U => Points (K).U + 0.2 * Gaussian, V => Points (K).V + 0.2 * Gaussian),
                               Certainty => 1.0);
            end if;
         end loop;
         Asked_Match (T);
         Matched (T, Points, Answers);
      end;
      Check (State (T) = Gone and then not Seen (T), "a thing nothing of which came back is still in the eye");
   end Nothing_Comes_Back;

   procedure Eye_Moved is
      --  The eye turned: the whole image moved seven pixels, the thing with
      --  it; the matcher finds its pixels and the table's alike.
      T : Track;
      function Shifted return Image is
         F : constant Image := Frame (20, 15);
         Data : Driver.Bytes.Byte_Array (1 .. 3 * W * H) := [others => 0];
         procedure Copy (RGB : Driver.Bytes.Byte_Array) is
         begin
            for R in 0 .. H - 1 loop
               for C in 0 .. W - 1 loop
                  for I in 1 .. 3 loop
                     Data (Driver.Bytes.Offset (3 * (R * W + C) + I)) :=
                       RGB (RGB'First + Driver.Bytes.Offset (3 * (R * W + (C + 7) mod W) + I - 1));
                  end loop;
               end loop;
            end loop;
         end Copy;
      begin
         Query (F, Copy'Access);
         return Create (W, H, Data);
      end Shifted;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 53);
      T := Start (Square (20, 15), Frame (20, 15), 0);
      Observe (T, Frame (20, 15), 1, Still => True);
      Observe (T, Frame (20, 15), 2, Still => True);
      Observe (T, Shifted, 3, Still => True);
      Observe (T, Shifted, 4, Still => True);
      Check (Wants_Match (T), "a thing whose eye moved was not looked for");
      declare
         Points  : constant Driver.Instrument.Point_Array := Match_Points (T);
         Answers : Driver.Instrument.Answer_Array (Points'Range);
      begin
         for K in Points'Range loop
            Answers (K) := (Found => True, To => (U => Points (K).U - 7.0, V => Points (K).V), Back => Points (K),
                            Certainty => 1.0);
         end loop;
         Asked_Match (T);
         Matched (T, Points, Answers);
      end;
      Check (Wants_Segment (T), "a thing whose eye moved was not found again");
      if Wants_Segment (T) then
         declare
            Around   : Driver.Instrument.Box;
            At_Point : Driver.Instrument.Pixel;
         begin
            Segment_Prompt (T, Around, At_Point);
            --  The whole image went seven pixels left, the thing with it.
            Check (abs (At_Point.U - 19.5) <= 1.0 and then Around.X0 >= 13.0 and then Around.X1 <= 25.0,
                   "the thing is not looked for where the eye's move took it");
         end;
      end if;
   end Eye_Moved;

   procedure Register is
   begin
      Driver.Tests.Register ("world.tracking.noise", "sensor noise loses a still thing more often than chance",
                             Noise_Keeps_It'Access);
      Driver.Tests.Register ("world.tracking.moved", "a thing that moved is kept, or not found again where it went",
                             Moved_And_Found'Access);
      Driver.Tests.Register ("world.tracking.gone", "a thing nothing of which comes back is kept in the eye",
                             Nothing_Comes_Back'Access);
      Driver.Tests.Register ("world.tracking.eye_moved",
                             "a thing whose eye moved is not found again where the move took it",
                             Eye_Moved'Access);
   end Register;

end Driver.World.Tracking.Tests;
