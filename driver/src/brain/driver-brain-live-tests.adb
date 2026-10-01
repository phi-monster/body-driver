with Driver.Tests;

package body Driver.Brain.Live.Tests is

   use Driver.Tests;

   procedure Point_On_It is
      Ok : Boolean;
      --  A ring around an empty middle, away from the picture's corner: the
      --  centroid is off the patch, and so is the origin.
      Ring : Driver.Images.Mask := Driver.Images.Create (7, 7);
   begin
      for I in 1 .. 5 loop
         Driver.Images.Include (Ring, I, 1);
         Driver.Images.Include (Ring, I, 5);
         Driver.Images.Include (Ring, 1, I);
         Driver.Images.Include (Ring, 5, I);
      end loop;
      declare
         P : constant Driver.Images.Pixel := Own_Point (Ring, Ok);
      begin
         Check (Ok and then Driver.Images.Contains (Ring, Natural (Real'Floor (P.U)), Natural (Real'Floor (P.V))),
                "the point of a ring lies on the ring, not in its empty middle");
      end;
      declare
         Unused : constant Driver.Images.Pixel := Own_Point (Driver.Images.Create (3, 3), Ok);
         pragma Unreferenced (Unused);
      begin
         Check (not Ok, "an empty patch has no point");
      end;
   end Point_On_It;

   procedure Register is
   begin
      Register ("brain.live.point", "the point that stands for a patch lies off it", Point_On_It'Access);
   end Register;

end Driver.Brain.Live.Tests;
