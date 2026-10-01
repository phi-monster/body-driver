separate (Act)
procedure Measure_Again (C : in out Context; F : Plug.Frame; Arm : Natural; Name : Unbounded_String; Predicted : Geom.V3; Got : out Boolean;
                         Forget_Old : Boolean := True) is
   Cam1 : constant Integer := Hand_Eye_Of (C, Integer (Arm));
   U, V : Long_Float;
   Seen, Whole, Edge, Mok : Boolean;
   Its_Name, Who : Unbounded_String;
   Sds : Floats;
   Spread : Long_Float;
   --  旧的那一份:没重新量成、又不该作废它的时候(它还在原处:手指合空了)放回去
   Old_Pts : constant Contact.V3_Vectors.Vector := C.Sil_Pts;
   Old_Valid : constant Boolean := C.Sil_Valid;
   Old_Name : constant Unbounded_String := C.Sil_Name;
   Old_Cam : constant Integer := C.Sil_Cam;
   Old_N : constant Geom.V3 := C.Sil_N;
   Old_P0 : constant Geom.V3 := C.Sil_P0;
   Old_Pitch : constant Long_Float := C.Sil_Pitch;
   Old_Err : constant Long_Float := C.Sil_Err;
   Old_H_Sd : constant Long_Float := C.Sil_H_Sd;
   Old_Rays : constant Geom.Sight_Vectors.Vector := C.Sil_Rays;
   procedure Done is
   begin
      Got := C.Sil_Valid and then C.Sil_Name = Name;
      if not Got and then not Forget_Old then
         C.Sil_Pts := Old_Pts; C.Sil_Valid := Old_Valid; C.Sil_Name := Old_Name; C.Sil_Cam := Old_Cam; C.Sil_N := Old_N; C.Sil_P0 := Old_P0;
         C.Sil_Pitch := Old_Pitch; C.Sil_Err := Old_Err; C.Sil_H_Sd := Old_H_Sd; C.Sil_Rays := Old_Rays;
      end if;
   end Done;
begin
   Got := False;
   C.Sil_Valid := False;
   if Cam1 < 0 or else Length (Name) = 0 then
      Done;
      return;
   end if;
   Retarget_Box (C, F, Natural (Cam1), Arm, Name, Predicted);
   Window_From_Outline (C, F, Natural (Cam1), Name);
   Geo_Track (C, F, Natural (Cam1), -1, U, V, Seen, Name);
   Slot_Whole (C, F, Natural (Cam1), -1, Whole, Edge, Its_Name, Name);
   declare
      Rays : constant Geom.Sight_Vectors.Vector := Sightlines_Now (C, F, Natural (Cam1), Arm, Name, Seen, Whole, U, V, Who, Sds);
      Pm : constant Geom.V3 := Geom.Meet (Rays, Mok, Spread);
   begin
      if not Mok then
         Done;
         return;
      end if;
      declare
         Pw : constant Geom.V3 := Plane_Point (C, Pm, Up_Dir (C), Say => False);
         Pw_Up_Sd : constant Long_Float := Geom.Meet_Sd (Rays, Sds, Pm, Up_Dir (C));
      begin
         C.Geo_Pw := Pw; C.Geo_Pw_Valid := True; C.Geo_Pw_Name := Name; C.Geo_Pw_Met := True; C.Geo_Pw_Up_Sd := Pw_Up_Sd;
         if Seen and then Whole then
            Take_Silhouette (C, F, Natural (Cam1), Arm, Name, Pw, Pw_Up_Sd);
         end if;
         for Cm in 0 .. C.Map.N_Cams - 1 loop
            if Cm /= Natural (Cam1) and then Cm < Natural (C.Geo.Length) and then Cm < Natural (F.Cams.Length) then
               declare
                  Bx2 : constant Integer := Boxed_By (C, Cm, Name);
               begin
                  if Bx2 >= 0 then
                     declare
                        B2 : constant Boxed_Thing := C.Boxed (Natural (Bx2));
                        Edge2 : constant Boolean := B2.X0 = 0 or else B2.Y0 = 0 or else B2.X1 + 1 >= F.Cams (Cm).W or else B2.Y1 + 1 >= F.Cams (Cm).H;
                     begin
                        if B2.Seen and then not Edge2 and then not Hand_Covers (C, F, Arm, Cm, B2) then
                           Take_Silhouette (C, F, Cm, Arm, Name, Pw, Pw_Up_Sd);
                        end if;
                     end;
                  end if;
               end;
            end if;
         end loop;
      end;
   end;
   Done;
end Measure_Again;
