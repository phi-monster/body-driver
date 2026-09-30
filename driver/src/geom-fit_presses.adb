separate (Geom)
function Fit_Presses (Eqs : Press_Eq_Vectors.Vector; Gate : Long_Float; View : Finger_View) return Press_Fit is
   Ai : Nat_Vectors.Vector;   --  对准这一瓣的那几下
   --  解出来的尖投回这只眼落不落在这一瓣的手指像素上(见 Finger_View)
   function On_Finger (X, Sd : V3) return Boolean is
      U, V : Long_Float;
      Front : Boolean;
   begin
      if View.Mask.Is_Empty or else View.W = 0 or else View.H = 0 or else Natural (View.Mask.Length) /= View.W * View.H then
         return True;
      end if;
      Cam_Pixel (View.G, X, U, V, Front);
      if not Front or else U < 0.0 or else V < 0.0 or else U >= Long_Float (View.W) or else V >= Long_Float (View.H) then
         return False;
      end if;
      declare
         Sp : constant Long_Float := View.G.F * Norm (Sd) / Long_Float'Max (Norm (X), Long_Float'Model_Small);   --  投回来的不确定度(像素)
         Reach : constant Long_Float := Stats.Z * Sp;
         Rp : constant Natural := Natural (Long_Float'Ceiling (Reach));
         Xc : constant Integer := Integer (Long_Float'Floor (U));
         Yc : constant Integer := Integer (Long_Float'Floor (V));
      begin
         for Y in Integer'Max (0, Yc - Rp) .. Integer'Min (View.H - 1, Yc + Rp) loop
            for Xx in Integer'Max (0, Xc - Rp) .. Integer'Min (View.W - 1, Xc + Rp) loop
               if View.Mask.Element (Y * View.W + Xx)
                 and then (Long_Float (Xx - Xc) ** 2 + Long_Float (Y - Yc) ** 2 <= Reach ** 2 or else (Xx = Xc and then Y = Yc))
               then
                  return True;
               end if;
            end loop;
         end loop;
         return False;
      end;
   end On_Finger;
   Max_Enum : constant := 16;   --  枚举的上限(次数:2^16 组;调用方一瓣最多压 8 下)
   Min_Set : constant := 4;     --  3 个未知数 + 1 条自己核(次数)
   function Dot (A, B : V3) return Long_Float is (A (0) * B (0) + A (1) * B (1) + A (2) * B (2));
   type Cand is record
      Fit : Press_Fit;
   end record;
   package Cand_Vectors is new Ada.Containers.Vectors (Natural, Cand);
   Cs : Cand_Vectors.Vector;
   Best_Size : Natural := 0;
   Res : Press_Fit;
   --  3×3 对称阵求逆(余子式);行列式相对 (迹/3)³ 太小 ⇒ 三个方向分不开(数值保护)
   procedure Inv3 (M : M3; Mi : out M3; Ok : out Boolean) is
      Det : Long_Float;
      Tr3 : constant Long_Float := (M (0, 0) + M (1, 1) + M (2, 2)) / 3.0;
   begin
      Mi (0, 0) := M (1, 1) * M (2, 2) - M (1, 2) * M (2, 1);
      Mi (0, 1) := M (0, 2) * M (2, 1) - M (0, 1) * M (2, 2);
      Mi (0, 2) := M (0, 1) * M (1, 2) - M (0, 2) * M (1, 1);
      Mi (1, 0) := M (1, 2) * M (2, 0) - M (1, 0) * M (2, 2);
      Mi (1, 1) := M (0, 0) * M (2, 2) - M (0, 2) * M (2, 0);
      Mi (1, 2) := M (0, 2) * M (1, 0) - M (0, 0) * M (1, 2);
      Mi (2, 0) := M (1, 0) * M (2, 1) - M (1, 1) * M (2, 0);
      Mi (2, 1) := M (0, 1) * M (2, 0) - M (0, 0) * M (2, 1);
      Mi (2, 2) := M (0, 0) * M (1, 1) - M (0, 1) * M (1, 0);
      Det := M (0, 0) * Mi (0, 0) + M (0, 1) * Mi (1, 0) + M (0, 2) * Mi (2, 0);
      Ok := Tr3 > 0.0 and then Det > 1.0e-12 * Tr3 ** 3;
      if Ok then
         for I in 0 .. 2 loop
            for J in 0 .. 2 loop
               Mi (I, J) := Mi (I, J) / Det;
            end loop;
         end loop;
      end if;
   end Inv3;
   function In_Mask (Mask, K : Natural) return Boolean is ((Mask / 2 ** K) mod 2 = 1);
   --  组里这几下(去掉 Skip 那一下;-1 = 不去)的最小二乘解
   procedure Solve (Mask : Natural; Skip : Integer; X : out V3; Mi : out M3; Ok : out Boolean) is
      M : M3 := [others => [others => 0.0]];
      V : V3 := [others => 0.0];
   begin
      for J in 0 .. Natural (Ai.Length) - 1 loop
         if In_Mask (Mask, J) and then J /= Skip then
            declare
               E : constant Press_Eq := Eqs (Ai (J));
            begin
               for R in 0 .. 2 loop
                  for S in 0 .. 2 loop
                     M (R, S) := M (R, S) + E.A (R) * E.A (S);
                  end loop;
                  V (R) := V (R) + E.A (R) * E.B;
               end loop;
            end;
         end if;
      end loop;
      Inv3 (M, Mi, Ok);
      X := (if Ok then Ap (Mi, V) else [0.0, 0.0, 0.0]);
   end Solve;
begin
   for I in 0 .. Natural (Eqs.Length) - 1 loop
      if Eqs (I).Aimed then
         Ai.Append (I);
      end if;
   end loop;
   if Natural (Ai.Length) < Min_Set or else Natural (Ai.Length) > Max_Enum then
      return Res;
   end if;
   for Mask in 1 .. 2 ** Natural (Ai.Length) - 1 loop
      declare
         K : Natural := 0;
      begin
         for J in 0 .. Natural (Ai.Length) - 1 loop
            if In_Mask (Mask, J) then
               K := K + 1;
            end if;
         end loop;
         if K >= Min_Set and then K >= Best_Size then
            declare
               Mi : M3;
               Inv_Ok : Boolean;
               F : Press_Fit;
               Good : Boolean := True;
            begin
               Solve (Mask, -1, F.X, Mi, Inv_Ok);
               Good := Inv_Ok;
               --  组里每一下:拿别的几下解、预测它
               for J in 0 .. Natural (Ai.Length) - 1 loop
                  exit when not Good;
                  if In_Mask (Mask, J) then
                     declare
                        Xo : V3;
                        Mo : M3;
                        Oo : Boolean;
                     begin
                        Solve (Mask, J, Xo, Mo, Oo);
                        if not Oo then
                           Good := False;
                        else
                           F.Worst := Long_Float'Max (F.Worst, abs (Dot (Eqs (Ai (J)).A, Xo) - Eqs (Ai (J)).B));
                           Good := F.Worst <= Gate;
                        end if;
                     end;
                  end if;
               end loop;
               if Good then
                  declare
                     Ss : Long_Float := 0.0;
                     Low : Long_Float := Long_Float'Last;
                     Inside : Boolean;
                  begin
                     for I in 0 .. Natural (Eqs.Length) - 1 loop
                        declare
                           R : constant Long_Float := Dot (Eqs (I).A, F.X) - Eqs (I).B;
                        begin
                           Inside := False;
                           for J in 0 .. Natural (Ai.Length) - 1 loop
                              if Ai (J) = I and then In_Mask (Mask, J) then
                                 Inside := True;
                              end if;
                           end loop;
                           if Inside then
                              F.Used.Append (I);
                              Ss := Ss + R * R;
                           else
                              Low := Long_Float'Min (Low, R);
                           end if;
                        end;
                     end loop;
                     F.Low := (if Low = Long_Float'Last then 0.0 else Low);
                     declare
                        Sig : constant Long_Float := Sqrt (Ss / Long_Float (K - 3));   --  自由度 = 下数 − 3 个未知数
                     begin
                        for R in 0 .. 2 loop
                           F.Sd (R) := Sig * Sqrt (Long_Float'Max (0.0, Mi (R, R)));
                        end loop;
                     end;
                     if F.Low >= -Gate and then On_Finger (F.X, F.Sd) then
                        F.Ok := True;
                        if K > Best_Size then
                           Cs.Clear;
                           Best_Size := K;
                        end if;
                        Cs.Append (Cand'(Fit => F));
                     end if;
                  end;
               end if;
            end;
         end if;
      end;
   end loop;
   if Cs.Is_Empty then
      return Res;
   end if;
   --  一样大的组不止一组:两组的解对两组里的每一下差都在 Gate 以内 = 同一个解(取预测差得最少的那组);差过 Gate = 认不出哪一下是坏的
   declare
      Pick : Natural := 0;
   begin
      for I in 0 .. Natural (Cs.Length) - 1 loop
         for J in I + 1 .. Natural (Cs.Length) - 1 loop
            declare
               Dx : constant V3 := [Cs (I).Fit.X (0) - Cs (J).Fit.X (0), Cs (I).Fit.X (1) - Cs (J).Fit.X (1), Cs (I).Fit.X (2) - Cs (J).Fit.X (2)];
            begin
               for U of Cs (I).Fit.Used loop
                  if abs Dot (Eqs (U).A, Dx) > Gate then
                     Res.Ambiguous := True;
                  end if;
               end loop;
               for U of Cs (J).Fit.Used loop
                  if abs Dot (Eqs (U).A, Dx) > Gate then
                     Res.Ambiguous := True;
                  end if;
               end loop;
            end;
         end loop;
         if Cs (I).Fit.Worst < Cs (Pick).Fit.Worst then
            Pick := I;
         end if;
      end loop;
      if Res.Ambiguous then
         return Res;
      end if;
      return Cs (Pick).Fit;
   end;
end Fit_Presses;
