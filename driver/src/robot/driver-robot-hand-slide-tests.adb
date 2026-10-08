with Ada.Numerics.Long_Elementary_Functions;
with Driver.Images;
with Driver.Tests;

package body Driver.Robot.Hand.Slide.Tests is

   use Ada.Numerics.Long_Elementary_Functions;
   use Driver.Numerics.Arrays;
   use Driver.Tests;

   Width  : constant := 160;
   Height : constant := 100;

   --  A finger: a wedge pointing right with its tip at (70, 50), dark and uniform,
   --  entering the picture from its left side, over a lit table with a texture of
   --  its own. The finger of the upper half of the picture stands Upper pixels
   --  along the way (1, 0) from where it stands free, that of the lower half Lower.

   function Inside (X, Y, Along : Real) return Boolean is
     (Y >= 20.0 and then Y <= 80.0 and then X - Along <= 70.0 - 0.8 * abs (Y - 50.0));

   function Table (X, Y : Real) return Real is
     (120.0 + 12.0 * Sin (0.7 * X) * Cos (0.5 * Y) + 6.0 * Sin (0.23 * X + 0.31 * Y));

   function Scene (Upper, Lower : Real; Slanted : Boolean := False) return Real_Array is
      Result : Real_Array (1 .. Width * Height);
   begin
      for R in 0 .. Height - 1 loop
         for C in 0 .. Width - 1 loop
            declare
               --  Each pixel's value is the mean of 16 samples, so an edge between pixels shows as one does.
               Sum   : Real := 0.0;
               Along : constant Real := (if Real (R) + 0.5 < 50.0 then Upper else Lower);
            begin
               for I in 0 .. 3 loop
                  for J in 0 .. 3 loop
                     declare
                        X : constant Real := Real (C) + (Real (I) + 0.5) / 4.0;
                        Y : constant Real := Real (R) + (Real (J) + 0.5) / 4.0;
                     begin
                        if (if Slanted
                            then Inside (X - 0.8 * Upper, Y - 0.6 * Upper, 0.0)   --  the whole finger moved Upper along (0.8, 0.6)
                            else Inside (X, Y, Along))
                        then
                           Sum := Sum + 20.0;
                        else
                           Sum := Sum + Table (X, Y);
                        end if;
                     end;
                  end loop;
               end loop;
               Result (1 + R * Width + C) := Sum / 16.0;
            end;
         end loop;
      end loop;
      return Result;
   end Scene;

   --  The patch of the free finger's tip region: the points of its edge from the column 40 on.
   function Free_Patch (U, V, Reach : Real) return Patch is
      Free   : constant Real_Array := Scene (0.0, 0.0);
      Lobe   : Driver.Images.Mask := Driver.Images.Create (Width, Height);
      Region : Driver.Images.Mask := Driver.Images.Create (Width, Height);
   begin
      for R in 0 .. Height - 1 loop
         for C in 0 .. Width - 1 loop
            if Free (1 + R * Width + C) < 60.0 then
               Driver.Images.Include (Lobe, C, R);
               if C >= 40 then
                  Driver.Images.Include (Region, C, R);
               end if;
            end if;
         end loop;
      end loop;
      return Take (Lobe, Region, Free, U, V, Reach);
   end Free_Patch;

   procedure Shifted_Finger is
      --  A finger that enters the picture from its side and is dark and uniform has its
      --  pixels alike wherever it slid to: only its edge tells where.
      Found : Shift;
   begin
      for Slid of Real_Vector'[0.0, 7.0, -5.0, 7.4, 19.0] loop
         Measure (Free_Patch (1.0, 0.0, 30.0), See (Scene (Slid, Slid), Width, Height), Found);
         Check (Found.Known, "a finger slid by" & Real'Image (Slid) & " pixels is not known");
         Check (abs (Found.By - Slid) < 0.5, "a finger slid by" & Real'Image (Slid) & " pixels is found at"
                & Real'Image (Found.By));
         Check (Found.Sigma < 1.0, "the shift of a finger slid as one piece is known to" & Real'Image (Found.Sigma));
         Check (Slid = 0.0 or else Found.Peak > Found.Still,
                "the free finger's place scores as well as that of a finger slid by" & Real'Image (Slid));
      end loop;
   end Shifted_Finger;

   procedure Pieces_Slid_Apart is
      --  The finger's upper part slid by 5 pixels and its lower part by 9: parts
      --  of it nearer the eye than others. They are not one shift, and the
      --  sigma says by how far.
      Found : Shift;
   begin
      Measure (Free_Patch (1.0, 0.0, 30.0), See (Scene (5.0, 9.0), Width, Height), Found);
      Check (Found.Known, "a finger slid by pieces is not known");
      Check (Found.By > 5.0 and then Found.By < 9.0, "a finger slid by 5 and 9 is found at" & Real'Image (Found.By));
      Check (Found.Sigma > 1.5, "a finger slid by 5 and 9 is known to" & Real'Image (Found.Sigma) & " pixels");
   end Pieces_Slid_Apart;

   procedure Slanted_Way is
      Found : Shift;
   begin
      Measure (Free_Patch (0.8, 0.6, 20.0), See (Scene (6.0, 6.0, Slanted => True), Width, Height), Found);
      Check (Found.Known and then abs (Found.By - 6.0) < 0.5, "a finger slid by 6 along (0.8, 0.6) is found at"
             & Real'Image (Found.By));
   end Slanted_Way;

   procedure Nothing_To_Match is
      --  A picture the finger is not in: the table's luma, which carries no finger.
      Found : Shift;
      Flat  : constant Real_Array (1 .. Width * Height) := [others => 120.0];
   begin
      Measure (Free_Patch (1.0, 0.0, 30.0), See (Flat, Width, Height), Found);
      Check (not Found.Known, "a finger is found in a picture without one at" & Real'Image (Found.By));
      --  And a finger that did not move, searched for as far as the picture is wide: the shifts that
      --  take more than half its pixels out of the picture are not compared.
      Measure (Free_Patch (1.0, 0.0, 100.0), See (Scene (0.0, 0.0), Width, Height), Found);
      Check (Found.Known and then abs (Found.By) < 0.5, "a finger is moved by" & Real'Image (Found.By)
             & " when it did not move and the search was wide");
   end Nothing_To_Match;

   procedure Register is
   begin
      Driver.Tests.Register ("hand.slide.shift", "a finger's pixels slid along their way are not found where they went",
                             Shifted_Finger'Access);
      Driver.Tests.Register ("hand.slide.pieces", "parts of a finger slid by different amounts are one shift, or a "
                             & "shift known better than they agree", Pieces_Slid_Apart'Access);
      Driver.Tests.Register ("hand.slide.way", "a finger slid along a slanted way is not found",
                             Slanted_Way'Access);
      Driver.Tests.Register ("hand.slide.none", "a finger is found where there is none, or moved when it was not",
                             Nothing_To_Match'Access);
   end Register;

end Driver.Robot.Hand.Slide.Tests;
