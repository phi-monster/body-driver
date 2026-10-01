with Driver.Tests;

package body Driver.Brain.Live.Tests is

   use Driver.Tests;

   procedure Runs is
      Ok : Boolean;
      --  A 4 x 3 picture: row 0 nothing, row 1 columns 1-2, row 2 columns 0-3.
      M  : constant Driver.Images.Mask := Region_Of_Runs (4, 3, [5, 2, 1, 4], Ok);
   begin
      Check (Ok and then Driver.Images.Count (M) = 6 and then Driver.Images.Contains (M, 1, 1)
             and then Driver.Images.Contains (M, 2, 1) and then not Driver.Images.Contains (M, 3, 1)
             and then Driver.Images.Contains (M, 0, 2), "runs alternate outside and inside, row by row");
      declare
         Short : constant Driver.Images.Mask := Region_Of_Runs (4, 3, [5, 2], Ok);
      begin
         Check (not Ok and then Driver.Images.Count (Short) = 2, "runs that do not cover the picture are not a region");
      end;
      declare
         Over : constant Driver.Images.Mask := Region_Of_Runs (4, 3, [5, 20], Ok);
      begin
         Check (not Ok and then Driver.Images.Count (Over) = 0, "runs past the picture are not a region");
      end;
   end Runs;

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
      Register ("brain.live.runs", "the instrument's runs become the wrong pixels", Runs'Access);
      Register ("brain.live.point", "the point that stands for a patch lies off it", Point_On_It'Access);
   end Register;

end Driver.Brain.Live.Tests;
