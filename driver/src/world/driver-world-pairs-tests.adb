with Ada.Numerics.Float_Random;
with Ada.Numerics.Long_Elementary_Functions;
with Driver.Tests;
with Driver.World.Tests;

package body Driver.World.Pairs.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;
   use type Driver.Images.Pixel;

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

   --  The top of a box, 0.1 on a side, 0.05 above a table at z = 0.
   Top_Height : constant Real := 0.05;
   function On_Top (X : Vec3) return Boolean is (abs (X (1) - 0.5) <= 0.05 and then abs (X (2)) <= 0.05);

   function Hit (Line : Ray_Estimate; Height : Real) return Vec3 is
      T : constant Real := (Height - Line.Origin.Mean (3)) / Line.Direction.Unit_Vector (3);
   begin
      return Line.Origin.Mean + T * Line.Direction.Unit_Vector;
   end Hit;

   procedure Box_Top is
      A : constant Driver.World.Tests.Pinhole :=
        Driver.World.Tests.Looking_At ([0.2, -0.3, 0.4], [0.5, 0.0, 0.0], 200.0, 160, 120, 0.1);
      B : constant Driver.World.Tests.Pinhole :=
        Driver.World.Tests.Looking_At ([0.2, 0.3, 0.4], [0.5, 0.0, 0.0], 200.0, 160, 120, 0.1);
      Own, Around : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 61);
      --  The region: the pixels of A that see the top; around it, those that
      --  see the table within as far again.
      for Pass in 1 .. 2 loop
         declare
            Points  : Driver.Instrument.Point_Array (1 .. Own + Around);
            Truth   : array (1 .. Own + Around) of Vec3;
            Answers : Driver.Instrument.Answer_Array (1 .. Own + Around);
            K_Own   : Natural := 0;
            K_Around : Natural := Own;
         begin
            for R in 0 .. 119 loop
               for C in 0 .. 159 loop
                  declare
                     P    : constant Driver.Images.Pixel := (U => Real (C) + 0.5, V => Real (R) + 0.5);
                     Line : constant Ray_Estimate := A.Ray (P);
                     Top  : constant Vec3 := Hit (Line, Top_Height);
                     Floor : constant Vec3 := Hit (Line, 0.0);
                  begin
                     if On_Top (Top) then
                        if Pass = 1 then
                           Own := Own + 1;
                        else
                           K_Own := K_Own + 1;
                           Points (K_Own) := P;
                           Truth (K_Own) := Top;
                        end if;
                     elsif abs (Floor (1) - 0.5) <= 0.1 and then abs (Floor (2)) <= 0.1 and then not On_Top (Floor) then
                        if Pass = 1 then
                           Around := Around + 1;
                        else
                           K_Around := K_Around + 1;
                           Points (K_Around) := P;
                           Truth (K_Around) := Floor;
                        end if;
                     end if;
                  end;
               end loop;
            end loop;
            if Pass = 2 then
               for K in Points'Range loop
                  declare
                     To      : Driver.Images.Pixel;
                     Visible : Boolean;
                  begin
                     B.Project (Truth (K), To, Visible);
                     --  One in eight of the top's matches is wrong by eight
                     --  pixels across, yet comes back.
                     if K <= Own and then K mod 8 = 0 then
                        To.U := To.U + 8.0;
                     end if;
                     Answers (K) := (Found     => Visible,
                                     To        => (U => To.U + Matcher_Sigma * Gaussian, V => To.V + Matcher_Sigma * Gaussian),
                                     Back      => (U => Points (K).U + Sqrt (2.0) * Matcher_Sigma * Gaussian,
                                                   V => Points (K).V + Sqrt (2.0) * Matcher_Sigma * Gaussian),
                                     Certainty => 1.0);
                  end;
               end loop;
               declare
                  Kept  : Match_Vectors.Vector;
                  Apart : Natural;
                  Far   : Natural;
                  Error : Real;
                  Wrong_Kept, Right_Off : Natural := 0;
                  Wrong : constant Natural := Own / 8;
               begin
                  Triangulate (A, B, Points, Own, Answers, Kept, Apart, Far, Error);
                  Check (abs (Error - Matcher_Sigma) < 0.2 * Matcher_Sigma,
                         "the matcher's error measured is" & Error'Image & " px, not the" & Matcher_Sigma'Image & " it made");
                  for M of Kept loop
                     declare
                        --  Find which pixel it was.
                        K : Natural := 0;
                     begin
                        for I in 1 .. Own loop
                           if Points (I) = M.In_First then
                              K := I;
                           end if;
                        end loop;
                        if K mod 8 = 0 then
                           Wrong_Kept := Wrong_Kept + 1;
                        else
                           declare
                              D : constant Vec3 := M.Point.Mean - Truth (K);
                           begin
                              --  Chi square with three degrees of freedom beyond 3 sigma: 14.2.
                              if D * (Inverse (M.Point.Covariance) * D) > 14.2 then
                                 Right_Off := Right_Off + 1;
                              end if;
                           end;
                        end if;
                     end;
                  end loop;
                  Check (Own > 100 and then Natural (Kept.Length) >= Own - Wrong - Own / 50,
                         "of" & Own'Image & " top pixels only" & Kept.Length'Image & " were kept");
                  Check (Wrong_Kept <= Wrong / 20 and then Apart >= Wrong - Wrong / 20,
                         "wrong matches were kept:" & Wrong_Kept'Image & " of" & Wrong'Image & ", apart" & Apart'Image);
                  Check (Right_Off <= Natural (Kept.Length) / 50,
                         Right_Off'Image & " points are off by more than their covariance allows");
               end;
            end if;
         end;
      end loop;
   end Box_Top;

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

   procedure Too_Far is
      --  Two eyes 0.1 apart, both looking down the x axis: what half the pixels
      --  see is half a metre off, what the other half see is a kilometre off,
      --  where the two sights part by a fiftieth of a pixel, under the
      --  matcher's error. Those are no place in the scene, the near ones are.
      A : constant Driver.World.Tests.Pinhole :=
        Driver.World.Tests.Looking_At ([0.0, -0.05, 0.0], [10.0, -0.05, 0.0], 200.0, 160, 120, 0.1);
      B : constant Driver.World.Tests.Pinhole :=
        Driver.World.Tests.Looking_At ([0.0, 0.05, 0.0], [10.0, 0.05, 0.0], 200.0, 160, 120, 0.1);
      Columns : constant := 16;
      Rows    : constant := 12;
      Points  : Driver.Instrument.Point_Array (1 .. Columns * Rows);
      Answers : Driver.Instrument.Answer_Array (Points'Range);
      Kept    : Match_Vectors.Vector;
      Apart, Far : Natural;
      Error   : Real;
      Near_Kept, Far_Kept, Near_Asked, Far_Asked : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 73);
      for R in 0 .. Rows - 1 loop
         for C in 0 .. Columns - 1 loop
            declare
               K : constant Positive := R * Columns + C + 1;
               P : constant Driver.Images.Pixel := (U => Real (10 * C) + 5.5, V => Real (10 * R) + 5.5);
               Sight : constant Ray_Estimate := A.Ray (P);
               Depth : constant Real := (if K mod 2 = 0 then 0.5 else 1000.0);
               Q     : Driver.Images.Pixel;
               Seen  : Boolean;
            begin
               Points (K) := P;
               B.Project (Sight.Origin.Mean + Depth * Sight.Direction.Unit_Vector, Q, Seen);
               Answers (K) := (Found     => Seen,
                               To        => (U => Q.U + Matcher_Sigma * Gaussian, V => Q.V + Matcher_Sigma * Gaussian),
                               Back      => (U => P.U + Sqrt (2.0) * Matcher_Sigma * Gaussian,
                                             V => P.V + Sqrt (2.0) * Matcher_Sigma * Gaussian),
                               Certainty => 1.0);
               if Seen and then K mod 2 = 1 then
                  Far_Asked := Far_Asked + 1;
               end if;
               if Seen and then K mod 2 = 0 then
                  Near_Asked := Near_Asked + 1;
               end if;
            end;
         end loop;
      end loop;
      Triangulate (A, B, Points, Points'Length, Answers, Kept, Apart, Far, Error);
      for M of Kept loop
         for K in Points'Range loop
            if Points (K) = M.In_First then
               if K mod 2 = 0 then
                  Near_Kept := Near_Kept + 1;
               else
                  Far_Kept := Far_Kept + 1;
               end if;
            end if;
         end loop;
      end loop;
      --  A far point whose two sights cross by the matcher's error looks placed:
      --  at the gate's tail, a few in a thousand. Without the test a third are.
      Check (Far_Kept <= Far_Asked / 20,
             Far_Kept'Image & " of" & Far_Asked'Image & " points a kilometre off were kept, with" & Far'Image
             & " too far");
      Check (Near_Asked > 50 and then Near_Kept >= Near_Asked - Near_Asked / 20,
             "of" & Near_Asked'Image & " points half a metre off only" & Near_Kept'Image & " were kept");
   end Too_Far;

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

   Back_Near : Boolean := False;
   --  Whether the wrong matches come back near where they set out, as a
   --  matcher can when it is consistently wrong.

   procedure Mostly_Unseen is
      --  Two eyes over the box top, the second seeing only three in ten of
      --  the pixels asked about: the others' matches land anywhere in its
      --  image and come back anywhere in the first, or, when Back_Near is
      --  set, within some six pixels of where they set out.
      A : constant Driver.World.Tests.Pinhole :=
        Driver.World.Tests.Looking_At ([0.2, -0.3, 0.4], [0.5, 0.0, 0.0], 200.0, 160, 120, 0.1);
      B : constant Driver.World.Tests.Pinhole :=
        Driver.World.Tests.Looking_At ([0.2, 0.3, 0.4], [0.5, 0.0, 0.0], 200.0, 160, 120, 0.1);
      Count : Natural := 0;
   begin
      Ada.Numerics.Float_Random.Reset (Gen, 71);
      for R in 0 .. 119 loop
         for C in 0 .. 159 loop
            Count := Count + Boolean'Pos (On_Top (Hit (A.Ray ((U => Real (C) + 0.5, V => Real (R) + 0.5)), Top_Height)));
         end loop;
      end loop;
      declare
         Points  : Driver.Instrument.Point_Array (1 .. Count);
         Truth   : array (1 .. Count) of Vec3;
         Seen    : array (1 .. Count) of Boolean;
         Answers : Driver.Instrument.Answer_Array (1 .. Count);
         K       : Natural := 0;
         Kept    : Match_Vectors.Vector;
         Apart   : Natural;
         Far     : Natural;
         Error   : Real;
         Wrong_Kept, Right_Kept, Right_Asked : Natural := 0;
      begin
         for R in 0 .. 119 loop
            for C in 0 .. 159 loop
               declare
                  P   : constant Driver.Images.Pixel := (U => Real (C) + 0.5, V => Real (R) + 0.5);
                  Top : constant Vec3 := Hit (A.Ray (P), Top_Height);
               begin
                  if On_Top (Top) then
                     K := K + 1;
                     Points (K) := P;
                     Truth (K) := Top;
                     Seen (K) := K mod 10 < 3;
                  end if;
               end;
            end loop;
         end loop;
         for I in Points'Range loop
            declare
               To      : Driver.Images.Pixel;
               Visible : Boolean;
            begin
               B.Project (Truth (I), To, Visible);
               if Seen (I) and then Visible then
                  Right_Asked := Right_Asked + 1;
                  Answers (I) := (Found     => True,
                                  To        => (U => To.U + Matcher_Sigma * Gaussian,
                                                V => To.V + Matcher_Sigma * Gaussian),
                                  Back      => (U => Points (I).U + Sqrt (2.0) * Matcher_Sigma * Gaussian,
                                                V => Points (I).V + Sqrt (2.0) * Matcher_Sigma * Gaussian),
                                  Certainty => 1.0);
               else
                  Answers (I) := (Found     => True,
                                  To        => (U => 160.0 * Uniform, V => 120.0 * Uniform),
                                  Back      =>
                                    (if Back_Near
                                     then (U => Points (I).U + 6.0 * Gaussian, V => Points (I).V + 6.0 * Gaussian)
                                     else (U => 160.0 * Uniform, V => 120.0 * Uniform)),
                                  Certainty => 1.0);
               end if;
            end;
         end loop;
         --  The region alone, so its own round trips must tell the error.
         Triangulate (A, B, Points, Points'Length, Answers, Kept, Apart, Far, Error);
         for M of Kept loop
            for I in Points'Range loop
               if Points (I) = M.In_First then
                  if Seen (I) then
                     Right_Kept := Right_Kept + 1;
                  else
                     Wrong_Kept := Wrong_Kept + 1;
                  end if;
               end if;
            end loop;
         end loop;
         Check (Right_Asked > 100 and then Right_Kept >= Right_Asked - Right_Asked / 20,
                "of" & Right_Asked'Image & " pixels the second eye sees only" & Right_Kept'Image & " were kept");
         --  A wrong match landing on the line its first sight draws in the second
         --  eye meets it whatever it shows: about one in forty here, by chance.
         --  No two eyes can tell those; any more than one in twenty, they could.
         Check (Wrong_Kept <= (Count - Right_Asked) / 20,
                Wrong_Kept'Image & " pixels the second eye does not see were kept, the matcher erring by"
                & Error'Image & " px");
      end;
   end Mostly_Unseen;

   procedure Consistently_Wrong is
      --  The same, the wrong matches coming back within some six pixels of
      --  where they set out: their round trips cannot tell them apart, their
      --  lines of sight can.
   begin
      Back_Near := True;
      Mostly_Unseen;
      Back_Near := False;
   end Consistently_Wrong;

   procedure Register is
   begin
      Driver.Tests.Register ("world.pairs.box", "two eyes' points are off their covariance, or wrong matches are kept",
                             Box_Top'Access);
      Driver.Tests.Register ("world.pairs.mixture",
                             "round trips of wrong matches, when most are wrong, are taken for the matcher's error",
                             Mixed_Round_Trips'Access);
      Driver.Tests.Register ("world.pairs.near_misses",
                             "matches lost near what they were asked are taken for the matcher's error",
                             Near_Misses'Access);
      Driver.Tests.Register ("world.pairs.too_far",
                             "points whose two sights are too near parallel to place are kept",
                             Too_Far'Access);
      Driver.Tests.Register ("world.pairs.unseen",
                             "matches into an eye that does not see most of what is asked are kept",
                             Mostly_Unseen'Access);
      Driver.Tests.Register ("world.pairs.consistent",
                             "wrong matches that come back near where they set out are kept",
                             Consistently_Wrong'Access);
   end Register;

end Driver.World.Pairs.Tests;
