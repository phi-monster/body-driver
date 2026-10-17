with Ada.Containers.Generic_Array_Sort;
with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Conventions;
with Driver.Tests;
with Driver.Uncertain;

package body Driver.World.Round_Trips.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Tests;

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

   Matcher_Sigma : constant Real := 0.3;   --  pixels, per coordinate of one matching

   function Uniform return Real is (Real (Ada.Numerics.Float_Random.Random (Gen)));

   procedure Mixed_Round_Trips is
      --  A thousand round trips, three in ten of right matches erring by a
      --  pixel per coordinate, the rest of wrong ones coming back anywhere in
      --  an image of 640 by 480; then a thousand all right, by half a pixel.
      N     : constant := 1000;
      Trips : Real_Array (1 .. 2 * N);
      Sigma, Right : Real;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 67);
      for I in 1 .. N loop
         if I mod 10 < 3 then
            Trips (2 * I - 1) := Gaussian;
            Trips (2 * I) := Gaussian;
         else
            Trips (2 * I - 1) := 640.0 * (Uniform - Uniform);
            Trips (2 * I) := 480.0 * (Uniform - Uniform);
         end if;
      end loop;
      Matcher_Error (Trips, 640.0 * 480.0, Sigma, Right);
      Check (abs (Sigma - 1.0) < 0.2 and then abs (Right - 300.0) < 60.0,
             "with most matches wrong the matcher's error is" & Sigma'Image & " px from" & Right'Image
             & " right ones, not 1 px from 300");
      for I in 1 .. 2 * N loop
         Trips (I) := 0.5 * Gaussian;
      end loop;
      Matcher_Error (Trips, 640.0 * 480.0, Sigma, Right);
      Check (abs (Sigma - 0.5) < 0.05 and then Right > 950.0,
             "with every match right the matcher's error is" & Sigma'Image & " px from" & Right'Image
             & " right ones, not 0.5 px from 1000");
   end Mixed_Round_Trips;

   procedure Near_Misses is
      --  Round trips as an eye matched into one that sees a tenth of it: forty
      --  right matches erring by 0.3 px per coordinate, sixty lost near what
      --  they were asked and coming back within some 8 px, the rest anywhere
      --  in an image of 640 by 480.
      N     : constant := 660;
      Trips : Real_Array (1 .. 2 * N);
      Sigma, Right : Real;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 71);
      for I in 1 .. N loop
         if I <= 40 then
            Trips (2 * I - 1) := Matcher_Sigma * Gaussian;
            Trips (2 * I) := Matcher_Sigma * Gaussian;
         elsif I <= 100 then
            Trips (2 * I - 1) := 8.0 * Gaussian;
            Trips (2 * I) := 8.0 * Gaussian;
         else
            Trips (2 * I - 1) := 640.0 * (Uniform - Uniform);
            Trips (2 * I) := 480.0 * (Uniform - Uniform);
         end if;
      end loop;
      Matcher_Error (Trips, 640.0 * 480.0, Sigma, Right);
      Check (abs (Sigma - Matcher_Sigma) < 0.25 * Matcher_Sigma and then abs (Right - 40.0) < 10.0,
             "with matches lost near what they were asked the matcher's error is" & Sigma'Image & " px from"
             & Right'Image & " right ones, not" & Matcher_Sigma'Image & " px from 40");
   end Near_Misses;

   procedure Long_Run_EM
     (Squared    : Real_Array;
      Dimensions : Positive;
      Measure    : Real;
      Sigma      : out Real;
      Right      : out Real;
      Passes     : in out Natural)
   is
      --  The mixture's maximum likelihood by EM alone, from every doubling
      --  rank of the errors' lengths, each run until its likelihood stops
      --  growing: the climb's reference.
      N      : constant Natural := Squared'Length;
      Best   : Real := Real'First;
      Wrong  : constant Real := 1.0 / Measure;
      K      : constant Real := Real (Dimensions);
      Sorted : Real_Array := Squared;
      Rank   : Positive := 1;

      procedure Fit (Start : Real) is
         S2   : Real := Start * Start;
         Pi   : Real := 0.5;
         Last : Real := Real'First;
      begin
         loop
            declare
               Weight : Real := 0.0;
               Spread : Real := 0.0;
               Like   : Real := 0.0;
               C      : constant Real := (2.0 * Ada.Numerics.Pi * S2) ** (-K / 2.0);
            begin
               Passes := Passes + 1;
               for D of Squared loop
                  declare
                     G : constant Real := C * Exp (-D / (2.0 * S2));
                     P : constant Real := Pi * G + (1.0 - Pi) * Wrong;
                     W : constant Real := Pi * G / P;
                  begin
                     Like := Like + Log (P);
                     Weight := Weight + W;
                     Spread := Spread + W * D;
                  end;
               end loop;
               exit when Like <= Last or else Weight <= 0.0 or else Spread <= 0.0;
               Last := Like;
               if Like > Best then
                  Best := Like;
                  Sigma := Sqrt (S2);
                  Right := Pi * Real (N);
               end if;
               S2 := Spread / (K * Weight);
               Pi := Weight / Real (N);
            end;
         end loop;
      end Fit;

      procedure Sort is new Ada.Containers.Generic_Array_Sort (Positive, Real, Real_Array);
   begin
      Sigma := Real'Last;
      Right := 0.0;
      Sort (Sorted);
      loop
         if Sorted (Sorted'First + Rank - 1) > 0.0 then
            Fit (Sqrt (Sorted (Sorted'First + Rank - 1) / K));
         end if;
         exit when Rank = N;
         Rank := Positive'Min (N, 2 * Rank);
      end loop;
   end Long_Run_EM;

   procedure Long_Run_Mixture
     (Squared    : Real_Array;
      Dimensions : Positive;
      Measure    : Real;
      Sigma      : out Real;
      Right      : out Real;
      Passes     : in out Natural)
   is
      --  The mixture fitted again in the window its own gate holds, as
      --  Mixture does, each fit by EM alone.
      Errors : Real_Array := Squared;
      Count  : Natural := Squared'Length;
      Window : Real := Measure;
   begin
      loop
         Long_Run_EM (Errors (Errors'First .. Errors'First + Count - 1), Dimensions, Window, Sigma, Right, Passes);
         exit when Right < 1.0 or else not (Sigma > 0.0 and then Sigma < Real'Last);
         declare
            Edge : constant Real :=
              Driver.Uncertain.Threshold
                (Driver.Uncertain.Vector_Gate (Dimensions, Natural (Real'Floor (Real (Dimensions) * Right))))
              * Sigma;
            Kept : Natural := 0;
         begin
            for I in Errors'First .. Errors'First + Count - 1 loop
               if Errors (I) <= Edge ** 2 then
                  Errors (Errors'First + Kept) := Errors (I);
                  Kept := Kept + 1;
               end if;
            end loop;
            exit when Kept = Count;
            Count := Kept;
            Window := Ball (Dimensions, Edge);
         end;
      end loop;
   end Long_Run_Mixture;

   procedure Climb_As_EM is
      --  The climb reaches the maximum EM run to its end reaches, within
      --  Unchanged_Fraction of the fit's own uncertainty (Driver.Conventions):
      --  a right match's sigma within that part of its standard error, the
      --  count of right ones within that part of a count's (its square root),
      --  by a tenth of EM's passes over the errors or fewer. At live sizes the
      --  whole fit, window by window; at the size a region asked pixel by pixel
      --  gives (55 000 matches), its last window, where the right ones fill it
      --  and the likelihood is flattest, as Mixture closes it on the climb.
      type Sample is (Mostly_Wrong, All_Right, Some_Wrong_Lines, Lines_Pixel_By_Pixel);
      UF : constant Real := Driver.Conventions.Unchanged_Fraction;

      procedure Agree (What : String; Dimensions : Positive; Sigma_EM, Right_EM, Sigma_Climb, Right_Climb : Real;
                       Passes_EM, Passes_Climb : Natural) is
         Sigma_SE : constant Real := Sigma_EM / Sqrt (2.0 * Real (Dimensions) * Right_EM);
      begin
         Check (abs (Sigma_Climb - Sigma_EM) <= UF * Sigma_SE and then abs (Right_Climb - Right_EM) <= UF * Sqrt (Right_EM),
                What & ": the climb gives a right match's sigma of" & Sigma_Climb'Image & " and" & Right_Climb'Image
                & " right ones, long-run EM" & Sigma_EM'Image & " and" & Right_EM'Image);
         Check (10 * Passes_Climb <= Passes_EM, What & ": the climb went over the errors" & Passes_Climb'Image
                & " times, long-run EM" & Passes_EM'Image);
      end Agree;
   begin
      for Which in Sample loop
         declare
            Dimensions : constant Positive := (if Which in Mostly_Wrong | All_Right then 2 else 1);
            N          : constant Positive :=
              (case Which is when Mostly_Wrong => 1000, when All_Right => 1500, when Some_Wrong_Lines => 1500,
                             when Lines_Pixel_By_Pixel => 55_000);
            Measure    : constant Real := (if Dimensions = 2 then 640.0 * 480.0 else 1600.0);
            Squared    : Real_Array (1 .. N);
            Sigma_EM, Right_EM, Sigma_Climb, Right_Climb : Real;
            Passes_EM, Passes_Climb : Natural := 0;
         begin
            Ada.Numerics.Float_Random.Reset (Gen, 79 + Sample'Pos (Which));
            for I in Squared'Range loop
               declare
                  Right_One : constant Boolean :=
                    (case Which is when Mostly_Wrong => I mod 10 < 3, when Some_Wrong_Lines => I mod 20 /= 0,
                                   when others => True);
               begin
                  if Dimensions = 2 then
                     Squared (I) :=
                       (if Right_One then (Matcher_Sigma * Gaussian) ** 2 + (Matcher_Sigma * Gaussian) ** 2
                        else (640.0 * (Uniform - 0.5)) ** 2 + (480.0 * (Uniform - 0.5)) ** 2);
                  else
                     Squared (I) := (if Right_One then (Matcher_Sigma * Gaussian) ** 2 else (800.0 * Uniform) ** 2);
                  end if;
               end;
            end loop;
            if Which = Lines_Pixel_By_Pixel then
               Mixture (Squared, Dimensions, Measure, Sigma_Climb, Right_Climb, Passes_Climb);
               declare
                  Edge   : constant Real :=
                    Driver.Uncertain.Threshold
                      (Driver.Uncertain.Vector_Gate (Dimensions, Natural (Real'Floor (Real (Dimensions) * Right_Climb))))
                    * Sigma_Climb;
                  Inside : Real_Array (1 .. N);
                  Count  : Natural := 0;
               begin
                  for D of Squared loop
                     if D <= Edge ** 2 then
                        Count := Count + 1;
                        Inside (Count) := D;
                     end if;
                  end loop;
                  Passes_Climb := 0;
                  Long_Run_EM (Inside (1 .. Count), Dimensions, Ball (Dimensions, Edge), Sigma_EM, Right_EM, Passes_EM);
                  Fit_Mixture (Inside (1 .. Count), Dimensions, Ball (Dimensions, Edge), Sigma_Climb, Right_Climb,
                               Passes_Climb);
               end;
            else
               Long_Run_Mixture (Squared, Dimensions, Measure, Sigma_EM, Right_EM, Passes_EM);
               Mixture (Squared, Dimensions, Measure, Sigma_Climb, Right_Climb, Passes_Climb);
            end if;
            Agree (Which'Image, Dimensions, Sigma_EM, Right_EM, Sigma_Climb, Right_Climb, Passes_EM, Passes_Climb);
         end;
      end loop;
   end Climb_As_EM;

   procedure Register is
   begin
      Driver.Tests.Register ("world.round_trips.climb",
                             "the mixture's climb ends off the maximum long-run EM reaches, or takes as "
                             & "many passes", Climb_As_EM'Access);
      Driver.Tests.Register ("world.round_trips.mixture",
                             "round trips of wrong matches, when most are wrong, are taken for the matcher's error",
                             Mixed_Round_Trips'Access);
      Driver.Tests.Register ("world.round_trips.near_misses",
                             "matches lost near what they were asked are taken for the matcher's error",
                             Near_Misses'Access);
   end Register;

end Driver.World.Round_Trips.Tests;
