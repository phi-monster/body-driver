separate (Act)
procedure Board_Free_Spots (C : Context; Lp : Geom.V3_Vectors.Vector; Tb : Floats; R : Long_Float; Deltas : out Geom.V3_Vectors.Vector) is
   N : constant Geom.V3 := C.Board_N;
   Nb : constant Natural := Natural (C.Board.Length);
   Na : constant Natural := Natural (C.Seen_Above.Length);
   --  板点在前,压之前看见的高出面的点(只挡)跟着,压之前看见、躺在面上、量得够细的点(C.Seen_On:当量过的桌面,同板点)在最后
   Nt : constant Natural := Nb + Na + Natural (C.Seen_On.Length);
   Fresh : constant Boolean := Natural (C.Board_Seen.Length) = Nb;   --  重找过(和板一一对应)
   On, Above, Tried : Bools;
   Hgt : Floats;
   Nn : Floats;                   --  每个量过的桌面上的板点离最近一个同类的多远(面内;别的点 = 0)
   Pp : Geom.V3_Vectors.Vector;   --  板点投到面上
   E1, E2 : Geom.V3 := [others => 0.0];   --  面内两根正交的轴(排方位用)
   package Sorting is new F64_Vectors.Generic_Sorting;
   function Gap (A, B : Geom.V3) return Long_Float is (Geom.Norm ([A (0) - B (0), A (1) - B (1), A (2) - B (2)]));
   --  落点 A0 那一圈(半径 R)整个在量过的桌面里:圈里每一处(按半个板点间距铺的格,采样,无量纲),离它两个板点间距以内那些量过的
   --  桌面上的板点把它围住 —— 按方位排开,最大的空档 < 180°(在那片里面);在那片的边上、边外、里面一块没点的洞里 = 空档 ≥ 180°。
   --  板点间距 = 离落点最近的 3 个(次数)量过的桌面上的板点各自离最近一个的中位(板自己量的;圈比间距小时圈里可能一个点都没有);
   --  两个间距 = 缺一个点照样围得住(倍数,无量纲);量过的桌面上不到 3 个点 ⇒ 量不出间距,不算
   function Inside (A0 : Geom.V3) return Boolean is
      Kn : constant := 3;
      Near : array (0 .. Kn - 1) of Long_Float := [others => Long_Float'Last];   --  最近几个的距离(从近到远)
      Near_S : array (0 .. Kn - 1) of Long_Float := [others => 0.0];            --  它们各自离最近一个同类的距离
      S : Long_Float;
      Pool : Geom.Nat_Vectors.Vector;   --  离落点 R + 两个间距以内的那些量过的桌面上的点
   begin
      for I in 0 .. Nt - 1 loop
         if On (I) and then Nn (I) > 0.0 then
            declare
               D : constant Long_Float := Gap (Pp (I), A0);
               K : Integer := Kn - 1;
            begin
               if D < Near (Kn - 1) then
                  while K > 0 and then Near (K - 1) > D loop
                     Near (K) := Near (K - 1); Near_S (K) := Near_S (K - 1);
                     K := K - 1;
                  end loop;
                  Near (K) := D; Near_S (K) := Nn (I);
               end if;
            end;
         end if;
      end loop;
      if Near (Kn - 1) = Long_Float'Last then
         return False;
      end if;
      --  三个的中位:排一下取中间那个
      declare
         Ns : Floats;
      begin
         for V of Near_S loop
            Ns.Append (V);
         end loop;
         Sorting.Sort (Ns);
         S := Ns (Kn / 2);
      end;
      for I in 0 .. Nt - 1 loop
         if On (I) and then Gap (Pp (I), A0) <= R + 2.0 * S then
            Pool.Append (I);
         end if;
      end loop;
      declare
         Kmax : constant := 64;   --  每条半径上最多铺这么多格(算力的上限,次数;板里两点几乎重合时间距会很小)
         Pitch : constant Long_Float := Long_Float'Max (0.5 * S, R / Long_Float (Kmax));
         M : constant Integer := Integer (Long_Float'Floor (R / Pitch));
      begin
         for Ia in -M .. M loop
            for Ib in -M .. M loop
               declare
                  Xa : constant Long_Float := Long_Float (Ia) * Pitch;
                  Xb : constant Long_Float := Long_Float (Ib) * Pitch;
                  X : constant Geom.V3 := [A0 (0) + Xa * E1 (0) + Xb * E2 (0), A0 (1) + Xa * E1 (1) + Xb * E2 (1), A0 (2) + Xa * E1 (2) + Xb * E2 (2)];
                  Angs : Floats;
                  Widest : Long_Float := 0.0;
               begin
                  if Xa * Xa + Xb * Xb <= R * R then
                     for I of Pool loop
                        declare
                           Q : constant Geom.V3 := [Pp (I) (0) - X (0), Pp (I) (1) - X (1), Pp (I) (2) - X (2)];
                           Qa : constant Long_Float := Q (0) * E1 (0) + Q (1) * E1 (1) + Q (2) * E1 (2);
                           Qb : constant Long_Float := Q (0) * E2 (0) + Q (1) * E2 (1) + Q (2) * E2 (2);
                           D2 : constant Long_Float := Qa * Qa + Qb * Qb;
                        begin
                           if D2 > 0.0 and then D2 <= 4.0 * S * S then
                              Angs.Append (Arctan (Qb, Qa));
                           end if;
                        end;
                     end loop;
                     if Angs.Is_Empty then
                        return False;
                     end if;
                     Sorting.Sort (Angs);
                     for K in 1 .. Natural (Angs.Length) - 1 loop
                        Widest := Long_Float'Max (Widest, Angs (K) - Angs (K - 1));
                     end loop;
                     Widest := Long_Float'Max (Widest, Angs (0) + 2.0 * Ada.Numerics.Pi - Angs (Natural (Angs.Length) - 1));
                     if Widest >= Ada.Numerics.Pi - 1.0e-9 then   --  正好 180°(点在两个板点连线上 = 那片的边)不算里面:数值上不许靠舍入定
                        return False;
                     end if;
                  end if;
               end;
            end loop;
         end loop;
      end;
      return True;
   end Inside;
   function Clear (Dl : Geom.V3) return Boolean is
      A0 : constant Geom.V3 := [Lp (0) (0) + Dl (0), Lp (0) (1) + Dl (1), Lp (0) (2) + Dl (2)];
   begin
      for I in 0 .. Nt - 1 loop
         if Above (I) then
            if Geom.Norm ([Pp (I) (0) - A0 (0), Pp (I) (1) - A0 (1), Pp (I) (2) - A0 (2)]) <= R then
               return False;
            end if;
            for J in 1 .. Natural (Lp.Length) - 1 loop
               declare
                  Aj : constant Geom.V3 := [Lp (J) (0) + Dl (0) - A0 (0), Lp (J) (1) + Dl (1) - A0 (1), Lp (J) (2) + Dl (2) - A0 (2)];
                  Ln : constant Long_Float := Geom.Norm (Aj);
                  Q : constant Geom.V3 := [Pp (I) (0) - A0 (0), Pp (I) (1) - A0 (1), Pp (I) (2) - A0 (2)];
                  Rho : constant Long_Float := (if Ln > 0.0 then Long_Float'Max (0.0, Long_Float'Min (Ln, (Q (0) * Aj (0) + Q (1) * Aj (1) + Q (2) * Aj (2)) / Ln)) else 0.0);
                  Side : constant Long_Float := (if Ln > 0.0 then Geom.Norm ([Q (0) - Rho * Aj (0) / Ln, Q (1) - Rho * Aj (1) / Ln, Q (2) - Rho * Aj (2) / Ln]) else Geom.Norm (Q));
               begin
                  if Side <= R and then Hgt (I) >= Rho * Tb (J) then
                     return False;
                  end if;
               end;
            end loop;
         end if;
      end loop;
      return Inside (A0);
   end Clear;
begin
   Deltas := Geom.V3_Vectors.Empty_Vector;
   if Lp.Is_Empty or else Nb = 0 or else Natural (Tb.Length) < Natural (Lp.Length) then
      return;
   end if;
   for I in 0 .. Nt - 1 loop
      declare
         S : constant Geom.Scene_Pt := (if I < Nb then C.Board (I) elsif I < Nb + Na then C.Seen_Above (I - Nb) else C.Seen_On (I - Nb - Na));
         H : constant Long_Float := (S.Pw (0) - C.Board_Pt (0)) * N (0) + (S.Pw (1) - C.Board_Pt (1)) * N (1) + (S.Pw (2) - C.Board_Pt (2)) * N (2);
         Cn : constant Geom.V3 := Geom.Ap (S.Cov, N);
         Tol : constant Long_Float := Plane_Tol (C, Cn (0) * N (0) + Cn (1) * N (1) + Cn (2) * N (2));
      begin
         On.Append (((I < Nb and then (not Fresh or else C.Board_Seen (I))) or else I >= Nb + Na) and then abs H <= Tol);
         Above.Append (H > Tol);
         Hgt.Append (H);
         Tried.Append (False);
         Pp.Append (Geom.V3'[S.Pw (0) - H * N (0), S.Pw (1) - H * N (1), S.Pw (2) - H * N (2)]);
      end;
   end loop;
   --  面内两根轴:法向叉上和它最不平行的那根坐标轴
   declare
      Ax : constant Geom.V3 := (if abs N (0) <= abs N (1) and then abs N (0) <= abs N (2) then [1.0, 0.0, 0.0]
                                elsif abs N (1) <= abs N (2) then [0.0, 1.0, 0.0] else [0.0, 0.0, 1.0]);
      Cx : constant Geom.V3 := [N (1) * Ax (2) - N (2) * Ax (1), N (2) * Ax (0) - N (0) * Ax (2), N (0) * Ax (1) - N (1) * Ax (0)];
      Cl : constant Long_Float := Geom.Norm (Cx);
   begin
      if Cl <= 0.0 then
         return;
      end if;
      E1 := [Cx (0) / Cl, Cx (1) / Cl, Cx (2) / Cl];
      E2 := [N (1) * E1 (2) - N (2) * E1 (1), N (2) * E1 (0) - N (0) * E1 (2), N (0) * E1 (1) - N (1) * E1 (0)];
   end;
   for I in 0 .. Nt - 1 loop
      declare
         Best : Long_Float := 0.0;
      begin
         if On (I) then
            Best := Long_Float'Last;
            for J in 0 .. Nt - 1 loop
               if J /= I and then On (J) then
                  Best := Long_Float'Min (Best, Gap (Pp (I), Pp (J)));
               end if;
            end loop;
            if Best = Long_Float'Last then
               Best := 0.0;
            end if;
         end if;
         Nn.Append (Best);
      end;
   end loop;
   if Clear ([0.0, 0.0, 0.0]) then
      Deltas.Append (Geom.V3'[0.0, 0.0, 0.0]);
   end if;
   loop
      declare
         Best : Integer := -1;
         Bd : Long_Float := Long_Float'Last;
      begin
         for I in 0 .. Nt - 1 loop
            if On (I) and then not Tried (I) then
               declare
                  D : constant Long_Float := Geom.Norm ([Pp (I) (0) - Lp (0) (0), Pp (I) (1) - Lp (0) (1), Pp (I) (2) - Lp (0) (2)]);
               begin
                  if D < Bd then
                     Bd := D; Best := I;
                  end if;
               end;
            end if;
         end loop;
         exit when Best < 0;
         Tried.Replace_Element (Natural (Best), True);
         declare
            Dl : constant Geom.V3 := [Pp (Natural (Best)) (0) - Lp (0) (0), Pp (Natural (Best)) (1) - Lp (0) (1), Pp (Natural (Best)) (2) - Lp (0) (2)];
         begin
            if Clear (Dl) then
               Deltas.Append (Dl);
            end if;
         end;
      end;
   end loop;
end Board_Free_Spots;
