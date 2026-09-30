separate (Act)
function Look_Points (C : Context; G : Geom.Cam_Geo; P0 : Plug.Arm_Pose; W, H : Natural;
                      Spot : Geom.V3; Far_Ends : Geom.V3_Vectors.Vector; R, Step_Px : Long_Float) return Instrument.Match_Vectors.Vector is
   Q : Instrument.Match_Vectors.Vector;
   Nb : constant Geom.V3 := C.Board_N;
   function Dot (P, Q : Geom.V3) return Long_Float is (P (0) * Q (0) + P (1) * Q (1) + P (2) * Q (2));
   function Free_Px (U, V : Long_Float) return Boolean is
     (U >= 0.0 and then V >= 0.0 and then U < Long_Float (W) and then V < Long_Float (H));
   O0 : constant Geom.V3 := Geom.Cam_Pos (G, P0);
   H0 : constant Long_Float := Dot ([O0 (0) - C.Board_Pt (0), O0 (1) - C.Board_Pt (1), O0 (2) - C.Board_Pt (2)], Nb);
   Sw : constant Long_Float := (if G.F > 0.0 then Long_Float'Max (1.0, Step_Px) * Long_Float'Max (0.0, H0) / G.F else 0.0);   --  铺点的间距(世界单位)
   procedure Ask (X : Geom.V3) is
      U, V : Long_Float;
      Front : Boolean;
   begin
      Geom.Project (G, P0, X, U, V, Front);
      if Front and then Free_Px (U, V) then
         Q.Append (Instrument.Match_Pt'(U => U, V => V, others => <>));
      end if;
   end Ask;
   --  从 Pa 到 Pb、两边各 R 的那一片(Pa = Pb ⇒ 以它为心、半径 R 的一圈)
   procedure Strip (Pa, Pb : Geom.V3) is
      Dv : constant Geom.V3 := [Pb (0) - Pa (0), Pb (1) - Pa (1), Pb (2) - Pa (2)];
      Hz : constant Long_Float := Dot (Dv, Nb);
      Hv : constant Geom.V3 := [Dv (0) - Hz * Nb (0), Dv (1) - Hz * Nb (1), Dv (2) - Hz * Nb (2)];
      Ln_S : constant Long_Float := Geom.Norm (Hv);
      --  面内两根轴:沿带子;没有长度 ⇒ 法向叉上和它最不平行的那根坐标轴
      Ax : constant Geom.V3 := (if abs Nb (0) <= abs Nb (1) and then abs Nb (0) <= abs Nb (2) then [1.0, 0.0, 0.0]
                                elsif abs Nb (1) <= abs Nb (2) then [0.0, 1.0, 0.0] else [0.0, 0.0, 1.0]);
      Cx : constant Geom.V3 := [Nb (1) * Ax (2) - Nb (2) * Ax (1), Nb (2) * Ax (0) - Nb (0) * Ax (2), Nb (0) * Ax (1) - Nb (1) * Ax (0)];
      T : constant Geom.V3 := (if Ln_S > 0.0 then [Hv (0) / Ln_S, Hv (1) / Ln_S, Hv (2) / Ln_S]
                               else [Cx (0) / Geom.Norm (Cx), Cx (1) / Geom.Norm (Cx), Cx (2) / Geom.Norm (Cx)]);
      Wv : constant Geom.V3 := [Nb (1) * T (2) - Nb (2) * T (1), Nb (2) * T (0) - Nb (0) * T (2), Nb (0) * T (1) - Nb (1) * T (0)];
      Na : constant Natural := Natural (Long_Float'Ceiling (Ln_S / Sw));
      Nr : constant Natural := Natural (Long_Float'Ceiling (R / Sw));
   begin
      for I in -Integer (Nr) .. Integer (Na + Nr) loop
         for J in -Integer (Nr) .. Integer (Nr) loop
            declare
               Al : constant Long_Float := Long_Float (I) * Sw;   --  沿带子离 Pa 多远
               Ac : constant Long_Float := Long_Float (J) * Sw;   --  离带子中线多远
               Along : constant Long_Float := Long_Float'Max (0.0, Long_Float'Min (Ln_S, Al));   --  离线段 Pa–Pb 多远(两头按圆)
            begin
               if (Al - Along) ** 2 + Ac ** 2 <= R * R then
                  Ask ([Pa (0) + Al * T (0) + Ac * Wv (0), Pa (1) + Al * T (1) + Ac * Wv (1), Pa (2) + Al * T (2) + Ac * Wv (2)]);
               end if;
            end;
         end loop;
      end loop;
   end Strip;
begin
   for Gyy in 0 .. Kinem.Gy - 1 loop
      for Gxx in 0 .. Kinem.Gx - 1 loop
         if Free_Px (Kinem.Grid_U (Gxx, W), Kinem.Grid_V (Gyy, H)) then
            Q.Append (Instrument.Match_Pt'(U => Kinem.Grid_U (Gxx, W), V => Kinem.Grid_V (Gyy, H), others => <>));
         end if;
      end loop;
   end loop;
   if Sw > 0.0 and then R > 0.0 then
      Strip (Spot, Spot);
      for Pb of Far_Ends loop
         Strip (Spot, Pb);
      end loop;
   end if;
   return Q;
end Look_Points;
