separate (Act)
procedure Init_Tracks (C : in out Context) is
begin
   --  死区一开始当作零(还没证据说哪个通道推不动),边走边学
   C.Dead.Clear;
   C.Reach_M.Clear;
   for K in 0 .. C.Map.Arms * Chan.Per_Arm loop
      C.Dead.Append (0.0);
      C.Reach_M.Append (0.0);   --  0 = 还没量过这根通道一推走几米
   end loop;
   C.Zones.Clear;
   for A in 0 .. C.Map.Arms - 1 loop
      for Cm in 0 .. C.Map.N_Cams - 1 loop
         declare
            Z : constant Zone.Hand_Zone := Zone_Of (C, A, Cm);
            T : Zone_Track;
         begin
            T.Valid := Z.Valid;
            T.Cu := Z.Cu; T.Cv := Z.Cv; T.Z := Z.Depth;
            T.Au := Z.A.Cu; T.Av := Z.A.Cv; T.Bu := Z.B.Cu; T.Bv := Z.B.Cv;
            T.Has_Lobes := Z.Valid and then Z.N_Lobes >= 1;
            T.Known := Z.Valid;
            if Z.Valid then
               T.Pieces (Chan.Per_Arm) := (True, Z.Cu, Z.Cv, (if Picture.Is_Nan (Z.Depth) then 0.0 else Z.Depth), Z.X0, Z.Y0, Z.X1, Z.Y1, Z.N_Lobes, Z.A.Cu, Z.A.Cv, Z.B.Cu, Z.B.Cv);
               T.Pieces_Known (Chan.Per_Arm) := True;
            end if;
            --  开机每个通道推过一下:跟着动的那块 = 这个通道带的零件(不长在这只手上的相机里才算)
            if Cam_Arm (C, Cm) /= Integer (A) then
               for K in 0 .. Chan.Per_Arm - 1 loop
                  declare
                     Pi : constant Natural := (A * Chan.Per_Arm + K) * C.Map.N_Cams + Cm;
                  begin
                     if Pi < Natural (C.Map.Parts.Length) and then C.Map.Parts (Pi).Valid then
                        declare
                           P : constant Selfmap.Part := C.Map.Parts (Pi);
                        begin
                           T.Pieces (K) := (True, P.Cu, P.Cv, 0.0, P.X0, P.Y0, P.X1, P.Y1, 1, P.Cu, P.Cv, 0.0, 0.0);
                           T.Pieces_Known (K) := True;
                        end;
                     end if;
                  end;
               end loop;
            end if;
            C.Zones.Append (T);
         end;
      end loop;
   end loop;
end Init_Tracks;
