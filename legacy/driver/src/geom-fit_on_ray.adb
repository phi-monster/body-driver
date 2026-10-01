separate (Geom)
function Fit_On_Ray (Eqs : Press_Eq_Vectors.Vector; D : V3; Gate : Long_Float) return Press_Fit is
   --  一点只差沿视线 D 多远(λ):每一下 A·(λ D) = B ⇒ a λ = B,a = A·D。对准它的几下里找对得上的最大的一组(同 Fit_Presses 的找法):
   --  组里每一下拿组里别的几下解的 λ 预测它,|a λ − B| ≤ Gate(只有一下的组没有别的几下可拿 ⇒ 不收:至少 2 下 = 1 个未知数 + 1 条自己核);
   --  一样大的组不止一组、解出来对组里哪一下差过 Gate ⇒ 认不出哪一下是坏的(Ambiguous)
   Ai : Nat_Vectors.Vector;   --  对准它的那几下
   Max_Enum : constant := 16;   --  枚举的上限(次数:2^16 组;调用方最多压几下)
   function Dot (P, Q : V3) return Long_Float is (P (0) * Q (0) + P (1) * Q (1) + P (2) * Q (2));
   function In_Mask (Mask, K : Natural) return Boolean is ((Mask / 2 ** K) mod 2 = 1);
   --  组里这几下(去掉 Skip 那一下;-1 = 不去)的最小二乘 λ;Ok = 分母不为零(组里有一下的视线不平行于面)
   procedure Solve (Mask : Natural; Skip : Integer; Lam, Saa : out Long_Float; Ok : out Boolean) is
      Sab : Long_Float := 0.0;
   begin
      Saa := 0.0;
      for J in 0 .. Natural (Ai.Length) - 1 loop
         if In_Mask (Mask, J) and then J /= Skip then
            declare
               E : constant Press_Eq := Eqs (Ai (J));
               A : constant Long_Float := Dot (E.A, D);
            begin
               Saa := Saa + A * A;
               Sab := Sab + A * E.B;
            end;
         end if;
      end loop;
      Ok := Saa > 0.0;
      Lam := (if Ok then Sab / Saa else 0.0);
   end Solve;
   Res : Press_Fit;
   type Cand is record
      Fit : Press_Fit;
      Lam : Long_Float := 0.0;
   end record;
   package Cand_Vectors is new Ada.Containers.Vectors (Natural, Cand);
   Cs : Cand_Vectors.Vector;
   Best_Size : Natural := 0;
begin
   for I in 0 .. Natural (Eqs.Length) - 1 loop
      if Eqs (I).Aimed then
         Ai.Append (I);
      end if;
   end loop;
   if Ai.Is_Empty or else Natural (Ai.Length) > Max_Enum or else Norm (D) = 0.0 then
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
         if K >= Best_Size then
            declare
               Lam, Saa : Long_Float;
               Good : Boolean;
               F : Press_Fit;
            begin
               Solve (Mask, -1, Lam, Saa, Good);
               Good := Good and then Lam > 0.0;   --  在眼前面(视线朝前)
               for J in 0 .. Natural (Ai.Length) - 1 loop
                  exit when not Good;
                  if In_Mask (Mask, J) then
                     declare
                        Lo, So : Long_Float;
                        Oo : Boolean;
                     begin
                        Solve (Mask, J, Lo, So, Oo);
                        if not Oo then
                           Good := False;
                        else
                           F.Worst := Long_Float'Max (F.Worst, abs (Dot (Eqs (Ai (J)).A, D) * Lo - Eqs (Ai (J)).B));
                           Good := F.Worst <= Gate;
                        end if;
                     end;
                  end if;
               end loop;
               if Good then
                  declare
                     Ss : Long_Float := 0.0;
                  begin
                     for J in 0 .. Natural (Ai.Length) - 1 loop
                        if In_Mask (Mask, J) then
                           F.Used.Append (Ai (J));
                           Ss := Ss + (Dot (Eqs (Ai (J)).A, D) * Lam - Eqs (Ai (J)).B) ** 2;
                        end if;
                     end loop;
                     declare
                        --  自由度 = 下数 − 1 个未知数;λ 的不确定度 = 噪声 / √Σa²,沿 D 摊到三个分量
                        Sig_L : constant Long_Float := Sqrt (Ss / Long_Float (K - 1)) / Sqrt (Saa);
                     begin
                        for R in 0 .. 2 loop
                           F.X (R) := Lam * D (R);
                           F.Sd (R) := Sig_L * abs D (R);
                        end loop;
                     end;
                     F.Ok := True;
                     if K > Best_Size then
                        Cs.Clear;
                        Best_Size := K;
                     end if;
                     Cs.Append (Cand'(Fit => F, Lam => Lam));
                  end;
               end if;
            end;
         end if;
      end;
   end loop;
   if Cs.Is_Empty then
      return Res;
   end if;
   declare
      Pick : Natural := 0;
   begin
      for I in 0 .. Natural (Cs.Length) - 1 loop
         for J in I + 1 .. Natural (Cs.Length) - 1 loop
            for U of Cs (I).Fit.Used loop
               if abs (Dot (Eqs (U).A, D) * (Cs (I).Lam - Cs (J).Lam)) > Gate then
                  Res.Ambiguous := True;
               end if;
            end loop;
            for U of Cs (J).Fit.Used loop
               if abs (Dot (Eqs (U).A, D) * (Cs (I).Lam - Cs (J).Lam)) > Gate then
                  Res.Ambiguous := True;
               end if;
            end loop;
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
end Fit_On_Ray;
