separate (Act)
procedure Seen_Above_Of (C : Context; G : Geom.Cam_Geo; P0, P1 : Plug.Arm_Pose; W, H : Natural; Qu, Qv, Mu, Mv, Bu, Bv : Floats;
                         Above : out Geom.Scene_Pt_Vectors.Vector; Matched, Tri : out Natural; Sig : out Long_Float;
                         On : access Geom.Scene_Pt_Vectors.Vector := null) is
   package Sorting is new F64_Vectors.Generic_Sorting;
   N : constant Geom.V3 := C.Board_N;
   O0 : constant Geom.V3 := Geom.Cam_Pos (G, P0);
   O1 : constant Geom.V3 := Geom.Cam_Pos (G, P1);
   Nq : constant Natural := Natural'Min (Natural'Min (Natural (Qu.Length), Natural (Qv.Length)),
                                         Natural'Min (Natural'Min (Natural (Mu.Length), Natural (Mv.Length)), Natural'Min (Natural (Bu.Length), Natural (Bv.Length))));
   Ok_M : Bools;
   Es : Floats;
begin
   Above := Geom.Scene_Pt_Vectors.Empty_Vector; Matched := 0; Tri := 0; Sig := 0.0;
   for I in 0 .. Nq - 1 loop
      declare
         Ok : constant Boolean := Mu (I) >= 0.0 and then Mv (I) >= 0.0 and then Mu (I) < Long_Float (W) and then Mv (I) < Long_Float (H)
           and then Geom.Round_Trip_Ok (Qu (I), Qv (I), Bu (I), Bv (I));
      begin
         Ok_M.Append (Ok);
         if Ok then
            Matched := Matched + 1;
            Es.Append (Sqrt ((Bu (I) - Qu (I)) ** 2 + (Bv (I) - Qv (I)) ** 2));
         end if;
      end;
   end loop;
   if Es.Is_Empty or else not (G.F > 0.0) then
      return;
   end if;
   Sorting.Sort (Es);
   Sig := Es (Natural (Es.Length) / 2) / Stats.Rayleigh_Median;
   if not (Sig > 0.0) then
      return;   --  往返分毫不差:量不出配点噪声 ⇒ 远近的不确定度也量不出,不判
   end if;
   for I in 0 .. Nq - 1 loop
      if Ok_M (I) then
         declare
            Ok0, Ok1, Okm, Front0, Front1, Okc : Boolean;
            D0 : constant Geom.V3 := Geom.Ray (G, P0, Qu (I), Qv (I), Ok0);
            D1 : constant Geom.V3 := Geom.Ray (G, P1, Mu (I), Mv (I), Ok1);
            Rays : Geom.Sight_Vectors.Vector;
            Sds : Floats;
            Spread : Long_Float;
            X : Geom.V3;
            U0, V0, U1, V1 : Long_Float;
            Cv : Geom.M3;
         begin
            if Ok0 and then Ok1 then
               Rays.Append (Geom.Sight'(O => O0, D => D0));
               Rays.Append (Geom.Sight'(O => O1, D => D1));
               X := Geom.Meet (Rays, Okm, Spread);
               if Okm then
                  Geom.Project (G, P0, X, U0, V0, Front0);
                  Geom.Project (G, P1, X, U1, V1, Front1);
                  if Front0 and then Front1
                    and then Sqrt ((U0 - Qu (I)) ** 2 + (V0 - Qv (I)) ** 2 + (U1 - Mu (I)) ** 2 + (V1 - Mv (I)) ** 2) <= Stats.Z * Sig
                  then
                     Tri := Tri + 1;
                     Sds.Append (Sig / G.F, Count => Rays.Length);   --  两条视线同一只眼、同一批配点
                     Cv := Geom.Meet_Cov (Rays, Sds, X, Okc);
                     if Okc then
                        declare
                           Hh : constant Long_Float := (X (0) - C.Board_Pt (0)) * N (0) + (X (1) - C.Board_Pt (1)) * N (1) + (X (2) - C.Board_Pt (2)) * N (2);
                           Cn : constant Geom.V3 := Geom.Ap (Cv, N);
                        begin
                           if Hh > Plane_Tol (C, Cn (0) * N (0) + Cn (1) * N (1) + Cn (2) * N (2)) then
                              Above.Append (Geom.Scene_Pt'(Pw => X, Cov => Cv, Sh => Sig, Views => Natural (Rays.Length), others => <>));
                           elsif On /= null and then abs Hh <= Plane_Tol (C, Cn (0) * N (0) + Cn (1) * N (1) + Cn (2) * N (2)) then
                              On.Append (Geom.Scene_Pt'(Pw => X, Cov => Cv, Sh => Sig, Views => Natural (Rays.Length), others => <>));
                           end if;
                        end;
                     end if;
                  end if;
               end if;
            end if;
         end;
      end if;
   end loop;
end Seen_Above_Of;
