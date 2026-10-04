with Driver.Tests;

package body Driver.Brain.Live.Tests is

   use Driver.Tests;

   procedure What_Most_Pixels_Are is
      --  A 10 by 10 patch, as the instrument gives it for a thing under a
      --  finger: the finger covers the patch's middle, 2 by 2 pixels of it.
      Patch  : Driver.Images.Mask := Driver.Images.Create (12, 12);
      Finger : Driver.Images.Mask := Driver.Images.Create (12, 12);
      Arm    : Driver.Images.Mask := Driver.Images.Create (12, 12);
      Edge   : Driver.Images.Mask := Driver.Images.Create (12, 12);
   begin
      for R in 1 .. 10 loop
         for C in 1 .. 10 loop
            Driver.Images.Include (Patch, C, R);
            --  The arm over six of the patch's ten rows; the edge over five.
            if R <= 6 then
               Driver.Images.Include (Arm, C, R);
            end if;
            if R <= 5 then
               Driver.Images.Include (Edge, C, R);
            end if;
         end loop;
      end loop;
      for R in 5 .. 6 loop
         for C in 5 .. 6 loop
            Driver.Images.Include (Finger, C, R);
         end loop;
      end loop;
      Check (not Mostly (Finger, Patch), "a thing whose middle a finger covers is not the body");
      Check (Mostly (Arm, Patch), "a patch six tenths on the body is mostly the body");
      Check (not Mostly (Edge, Patch), "half of a patch is not most of it");
      Check (not Mostly (Arm, Driver.Images.Create (12, 12)), "an empty patch is mostly nothing");
   end What_Most_Pixels_Are;

   procedure Register is
   begin
      Register ("brain.live.mostly", "a patch is taken for what one point of it is, not what most of it is",
                What_Most_Pixels_Are'Access);
   end Register;

end Driver.Brain.Live.Tests;
