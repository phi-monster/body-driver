with Driver.Alignment;
with Driver.Images;

package body Driver.World.Sight is

   use Driver.Numerics.Arrays;
   use type Driver.Observations.Camera_Id;

   function Known_Pose (P : Pose_Estimate) return Boolean is
     (P.Position_Covariance (1, 1) < Real'Last and then P.Rotation_Covariance (1, 1) < Real'Last);

   --  How far along the line of sight through the middle of the camera's picture it meets the table, and the angle a
   --  pixel spans there; 0 when the line does not meet it in front or the camera is not measured.
   procedure Middle (C : Driver.World.Cameras.Camera'Class; Table : Driver.Geometry.Plane_Estimate;
                     Depth, Pixel : out Real) is
      Px  : constant Driver.World.Cameras.Pixel := (U => Real (C.Width) / 2.0, V => Real (C.Height) / 2.0);
      Ray : constant Ray_Estimate := C.Ray (Px);
   begin
      Depth := 0.0;
      Pixel := Driver.World.Cameras.Radians_Per_Pixel (C, Px);
      if Ray.Direction.Sigma < Real'Last and then Pixel < Real'Last and then Pixel > 0.0 then
         declare
            Facing : constant Real := Table.Normal * Ray.Direction.Unit_Vector;
         begin
            if Facing /= 0.0 then
               Depth := Real'Max (0.0, (Table.Normal * (Table.Centre - Ray.Origin.Mean)) / Facing);
            end if;
         end;
      end if;
   end Middle;

   procedure Keep_View (B : in out Bank; Camera_Of : Camera_Function; Table : Driver.Geometry.Plane_Estimate;
                        E : Eye_Id; O : Observation) is
   begin
      if not Driver.Geometry.Known (Table) or else E > O.Images.Last_Index or else not Driver.Observations.Has_Image (O, E)
      then
         return;
      end if;
      declare
         Bare : Observation := O;
      begin
         for K in Bare.Images.First_Index .. Bare.Images.Last_Index loop
            if K /= E then
               Bare.Images.Replace_Element (K, Driver.Images.No_Image);
            end if;
         end loop;
         declare
            Held : constant Observation_Holders.Holder := Observation_Holders.To_Holder (Bare);
            Ref  : constant Observation_Holders.Constant_Reference_Type := Held.Constant_Reference;
            Cam  : constant Driver.World.Cameras.Camera'Class := Camera_Of (E, Ref.Element);
            Pose : constant Pose_Estimate := Cam.Pose;
            Depth, Pixel : Real;
         begin
            Middle (Cam, Table, Depth, Pixel);
            if Known_Pose (Pose) and then Depth > 0.0 then
               declare
                  Stood   : constant Vec3 := Pose.Pose.Translation;
                  Closest : Natural := 0;
                  Least   : Real := Real'Last;
               begin
                  for K in B.Kept.First_Index .. B.Kept.Last_Index loop
                     if B.Kept (K).Eye = E and then abs (B.Kept (K).Stood - Stood) < Least then
                        Least := abs (B.Kept (K).Stood - Stood);
                        Closest := K;
                     end if;
                  end loop;
                  --  Within a pixel's parallax at the table of a view kept before: the same view, seen again.
                  if Closest > 0 and then Least / Depth < Pixel then
                     B.Kept.Replace_Element (Closest, (Eye => E, Seen => Held, Stood => Stood));
                  else
                     B.Kept.Append (View_Record'(Eye => E, Seen => Held, Stood => Stood));
                  end if;
               end;
            end if;
         end;
      end;
   end Keep_View;

   procedure Keep (B : in out Bank; Eyes : Natural; Camera_Of : Camera_Function; Table : Driver.Geometry.Plane_Estimate;
                   O : Observation) is
   begin
      for E in 1 .. Eye_Id'Base (Eyes) loop
         Keep_View (B, Camera_Of, Table, E, O);
      end loop;
   end Keep;

   function Views (B : Bank) return Natural is (Natural (B.Kept.Length));

   function Views_Of (B : Bank; E : Eye_Id) return Natural is
      N : Natural := 0;
   begin
      for V of B.Kept loop
         if V.Eye = E then
            N := N + 1;
         end if;
      end loop;
      return N;
   end Views_Of;

   procedure Place
     (B         : Bank;
      Camera_Of : Camera_Function;
      Table     : Driver.Geometry.Plane_Estimate;
      E         : Eye_Id;
      O         : Observation;
      Queries   : Driver.Robot.Surfaces.Pixel_Array;
      Columns   : Positive;
      Result    : out Driver.Robot.Surfaces.Placed_Array;
      Counts    : out Driver.Robot.Surfaces.Placing_Counts)
   is
      Held_Now : constant Observation_Holders.Holder := Observation_Holders.To_Holder (O);
      Ref_Now  : constant Observation_Holders.Constant_Reference_Type := Held_Now.Constant_Reference;
      Now_Cam  : constant Driver.World.Cameras.Camera'Class := Camera_Of (E, Ref_Now.Element);
      Now_Pose : constant Pose_Estimate := Now_Cam.Pose;
      Depth, Pixel : Real;
      --  The views used, by their place in the bank: lens K + 1 is the camera of view Used (K); lens 1 the eye now.
      Used     : array (1 .. Natural (B.Kept.Length)) of Positive := [others => 1];
      Count    : Natural := 0;

      function Camera_Of_Lens (Lens : Positive) return Driver.World.Cameras.Camera'Class is
        (if Lens = 1 then Now_Cam
         else Camera_Of (B.Kept (Used (Lens - 1)).Eye, B.Kept (Used (Lens - 1)).Seen.Constant_Reference.Element));

      function Lens_Ray (Lens : Positive; U, V : Real) return Vec3 is (Camera_Of_Lens (Lens).Ray_In_Eye (U, V));

      procedure Lens_Project (Lens : Positive; P : Vec3; U, V : out Real; In_Front : out Boolean) is
      begin
         Camera_Of_Lens (Lens).Project_In_Eye (P, U, V, In_Front);
      end Lens_Project;
   begin
      for R of Result loop
         R := (others => <>);
      end loop;
      Counts := (others => 0);
      if not Driver.Geometry.Known (Table) or else E > O.Images.Last_Index
        or else not Driver.Observations.Has_Image (O, E) or else not Known_Pose (Now_Pose)
      then
         return;
      end if;
      Middle (Now_Cam, Table, Depth, Pixel);
      if Depth <= 0.0 then
         return;
      end if;
      --  Every view but those of this eye standing where it stands now, within a pixel's parallax at the table.
      for K in B.Kept.First_Index .. B.Kept.Last_Index loop
         if B.Kept (K).Eye /= E or else abs (B.Kept (K).Stood - Now_Pose.Pose.Translation) / Depth >= Pixel then
            Count := Count + 1;
            Used (Count) := K;
         end if;
      end loop;
      if Count = 0 then
         return;
      end if;
      declare
         Elsewhere : Driver.Robot.Surfaces.View_Array (1 .. Count);
      begin
         for K in 1 .. Count loop
            declare
               V   : View_Record renames B.Kept (Used (K));
               Cam : constant Driver.World.Cameras.Camera'Class := Camera_Of_Lens (K + 1);
            begin
               Elsewhere (K) := (Pose    => Cam.Pose.Pose,
                                 Picture => Driver.Alignment.Pyramid_Of (V.Seen.Constant_Reference.Element.Images (V.Eye)),
                                 Lens    => K + 1,
                                 Noise   => Cam.Noise);
            end;
         end loop;
         Driver.Robot.Surfaces.Place
           ((Pose => Now_Pose.Pose, Picture => Driver.Alignment.Pyramid_Of (O.Images (E)), Lens => 1,
             Noise => Now_Cam.Noise),
            Elsewhere, Queries, Columns, Table, Lens_Ray'Unrestricted_Access, Lens_Project'Unrestricted_Access, Result,
            Counts);
      end;
   end Place;

end Driver.World.Sight;
