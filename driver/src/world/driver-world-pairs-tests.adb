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
                  Wrong_Kept, Right_Off : Natural := 0;
                  Wrong : constant Natural := Own / 8;
               begin
                  Triangulate (A, B, Points, Own, Answers, Kept, Apart);
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

   procedure Register is
   begin
      Driver.Tests.Register ("world.pairs.box", "two eyes' points are off their covariance, or wrong matches are kept",
                             Box_Top'Access);
   end Register;

end Driver.World.Pairs.Tests;
