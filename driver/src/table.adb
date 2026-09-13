with Ada.Numerics.Long_Elementary_Functions; use Ada.Numerics.Long_Elementary_Functions;
package body Table is
   function Norm3 (V : Vec3) return Long_Float is (Sqrt (V (0) * V (0) + V (1) * V (1) + V (2) * V (2)));

   function Norm (A : Vec; N : Natural) return Long_Float is
      S : Long_Float := 0.0;
   begin
      for I in 0 .. Natural'Min (N, Max_Ch) - 1 loop
         S := S + A (I) * A (I);
      end loop;
      return Sqrt (S);
   end Norm;

   procedure Reset (E : in out Effect; N : Natural; P0 : Long_Float) is
   begin
      E.N := Natural'Min (N, Max_Ch);
      E.B := [others => [others => 0.0]];
      E.P := [others => [others => 0.0]];
      for I in 0 .. E.N - 1 loop
         E.P (I, I) := P0;
      end loop;
      E.Free_Res := 0.0; E.Null_Res := 0.0; E.Null_Wins := 0; E.Updates := 0; E.Last_Pred_Err := 0.0;
   end Reset;

   procedure Set_Prior (E : in out Effect; Ch : Natural; P0 : Long_Float) is
   begin
      if Ch < E.N then
         E.P (Ch, Ch) := P0;
      end if;
   end Set_Prior;

   procedure Set_Spread (E : in out Effect; Ch : Natural; N : Natural; S : Vec3) is
   begin
      if Ch <= Ch_Index'Last then
         E.Reps (Ch) := N;
         for R in 0 .. Rows - 1 loop
            E.Scatter (Ch, R) := S (R);
         end loop;
      end if;
   end Set_Spread;

   procedure Set_Col (E : in out Effect; Ch : Natural; D : Vec3) is
   begin
      if Ch < E.N then
         for R in 0 .. Rows - 1 loop
            E.B (Ch, R) := D (R);
         end loop;
      end if;
   end Set_Col;

   function Col (E : Effect; Ch : Natural) return Vec3 is
      V : Vec3 := Zero3;
   begin
      if Ch < E.N then
         for R in 0 .. Rows - 1 loop
            V (R) := E.B (Ch, R);
         end loop;
      end if;
      return V;
   end Col;

   function Predict (E : Effect; A : Vec) return Vec3 is
      V : Vec3 := Zero3;
   begin
      for R in 0 .. Rows - 1 loop
         for I in 0 .. E.N - 1 loop
            V (R) := V (R) + E.B (I, R) * A (I);
         end loop;
      end loop;
      return V;
   end Predict;

   procedure Update (E : in out Effect; A : Vec; Dy : Vec3; Motion_Floor, Cmd_Floor : Long_Float) is
      Pred : constant Vec3 := Predict (E, A);
      Err : Vec3;
      Pa : Vec := Zero_Vec;
      Denom : Long_Float := E.Lambda;
      K : Vec := Zero_Vec;
   begin
      for R in 0 .. Rows - 1 loop
         Err (R) := Dy (R) - Pred (R);
      end loop;
      E.Free_Res := Norm3 (Err);
      E.Null_Res := Norm3 (Dy);
      E.Last_Pred_Err := E.Free_Res;
      --  责任:命令真的发出去了(超过本体噪声)、走的表错得超过画面噪声、而"什么都不动"反而更准 ⇒ 这一步像顶住了
      if Norm (A, E.N) > Cmd_Floor and then E.Free_Res > Motion_Floor and then E.Null_Res < E.Free_Res and then E.Updates >= E.N then
         E.Null_Wins := E.Null_Wins + 1;
      else
         E.Null_Wins := 0;
      end if;
      if Norm (A, E.N) <= Cmd_Floor then
         return;    --  没动就没有信息,表不动
      end if;
      --  递推最小二乘:K = P a / (λ + aᵀ P a);B += K (dy − B a)ᵀ;P = (P − K aᵀ P) / λ
      for I in 0 .. E.N - 1 loop
         for J in 0 .. E.N - 1 loop
            Pa (I) := Pa (I) + E.P (I, J) * A (J);
         end loop;
         Denom := Denom + A (I) * Pa (I);
      end loop;
      if not (Denom > 1.0e-18) then
         return;
      end if;
      for I in 0 .. E.N - 1 loop
         K (I) := Pa (I) / Denom;
      end loop;
      for I in 0 .. E.N - 1 loop
         for R in 0 .. Rows - 1 loop
            E.B (I, R) := E.B (I, R) + K (I) * Err (R);
         end loop;
      end loop;
      declare
         Ap : Vec := Zero_Vec;   --  aᵀ P
      begin
         for J in 0 .. E.N - 1 loop
            for I in 0 .. E.N - 1 loop
               Ap (J) := Ap (J) + A (I) * E.P (I, J);
            end loop;
         end loop;
         for I in 0 .. E.N - 1 loop
            for J in 0 .. E.N - 1 loop
               E.P (I, J) := (E.P (I, J) - K (I) * Ap (J)) / E.Lambda;
            end loop;
         end loop;
      end;
      E.Updates := E.Updates + 1;
   end Update;

   function Blocked (E : Effect) return Boolean is (E.Null_Wins >= 2);

   function Spread (E : Effect) return Long_Float is
      S : Long_Float := 0.0;
   begin
      if E.N = 0 then
         return 0.0;
      end if;
      for I in 0 .. E.N - 1 loop
         S := S + E.P (I, I);
      end loop;
      return S / Long_Float (E.N);
   end Spread;

   procedure Solve (Terms : Term_Vectors.Vector; N : Natural; Cap : Vec; Active : Mask; Damp : Vec;
                    A : out Vec; Ok : out Boolean) is
      Nn : constant Natural := Natural'Min (N, Max_Ch);
      G : Cov := [others => [others => 0.0]];
      Hv : Vec := Zero_Vec;
      Fixed : Mask := [others => False];
   begin
      A := Zero_Vec;
      Ok := False;
      if Nn = 0 or else Terms.Is_Empty then
         return;
      end if;
      for T of Terms loop
         for R in 0 .. Rows - 1 loop
            if T.W (R) > 0.0 then
               for I in 0 .. Nn - 1 loop
                  declare
                     Bi : constant Long_Float := (if I < T.E.N then T.E.B (I, R) else 0.0);
                  begin
                     Hv (I) := Hv (I) + T.W (R) * Bi * T.Err (R);
                     for J in 0 .. Nn - 1 loop
                        G (I, J) := G (I, J) + T.W (R) * Bi * (if J < T.E.N then T.E.B (J, R) else 0.0);
                     end loop;
                  end;
               end loop;
            end if;
         end loop;
      end loop;
      for I in 0 .. Nn - 1 loop
         G (I, I) := G (I, I) + Damp (I);
         if not Active (I) then
            Fixed (I) := True;
            A (I) := 0.0;
         end if;
      end loop;
      --  投影迭代:解自由通道;越限的夹到限上、固定住、把它的贡献搬到右边;再解。三轮(次数,无量纲)。
      for Round in 1 .. 3 loop
         declare
            Idx : array (Ch_Index) of Natural := [others => 0];
            M : Natural := 0;
            Sys : Cov := [others => [others => 0.0]];
            Rhs : Vec := Zero_Vec;
         begin
            for I in 0 .. Nn - 1 loop
               if not Fixed (I) then
                  Idx (M) := I;
                  M := M + 1;
               end if;
            end loop;
            if M = 0 then
               Ok := True;
               return;
            end if;
            for P in 0 .. M - 1 loop
               declare
                  I : constant Natural := Idx (P);
               begin
                  Rhs (P) := Hv (I);
                  for J in 0 .. Nn - 1 loop
                     if Fixed (J) then
                        Rhs (P) := Rhs (P) - G (I, J) * A (J);
                     end if;
                  end loop;
                  for Q in 0 .. M - 1 loop
                     Sys (P, Q) := G (I, Idx (Q));
                  end loop;
               end;
            end loop;
            --  高斯消元(部分主元)
            for C in 0 .. M - 1 loop
               declare
                  Piv : Natural := C;
               begin
                  for R in C + 1 .. M - 1 loop
                     if abs Sys (R, C) > abs Sys (Piv, C) then
                        Piv := R;
                     end if;
                  end loop;
                  if abs Sys (Piv, C) < 1.0e-15 then
                     return;
                  end if;
                  if Piv /= C then
                     for Q in 0 .. M - 1 loop
                        declare
                           Tmp : constant Long_Float := Sys (C, Q);
                        begin
                           Sys (C, Q) := Sys (Piv, Q);
                           Sys (Piv, Q) := Tmp;
                        end;
                     end loop;
                     declare
                        Tmp : constant Long_Float := Rhs (C);
                     begin
                        Rhs (C) := Rhs (Piv);
                        Rhs (Piv) := Tmp;
                     end;
                  end if;
                  for R in C + 1 .. M - 1 loop
                     declare
                        F : constant Long_Float := Sys (R, C) / Sys (C, C);
                     begin
                        if F /= 0.0 then
                           for Q in C .. M - 1 loop
                              Sys (R, Q) := Sys (R, Q) - F * Sys (C, Q);
                           end loop;
                           Rhs (R) := Rhs (R) - F * Rhs (C);
                        end if;
                     end;
                  end loop;
               end;
            end loop;
            declare
               X : Vec := Zero_Vec;
               Any_Clamped : Boolean := False;
            begin
               for R in reverse 0 .. M - 1 loop
                  declare
                     S : Long_Float := Rhs (R);
                  begin
                     for Q in R + 1 .. M - 1 loop
                        S := S - Sys (R, Q) * X (Q);
                     end loop;
                     X (R) := S / Sys (R, R);
                  end;
               end loop;
               for P in 0 .. M - 1 loop
                  declare
                     I : constant Natural := Idx (P);
                     C : constant Long_Float := abs Cap (I);
                  begin
                     if X (P) > C then
                        A (I) := C; Fixed (I) := True; Any_Clamped := True;
                     elsif X (P) < -C then
                        A (I) := -C; Fixed (I) := True; Any_Clamped := True;
                     else
                        A (I) := X (P);
                     end if;
                  end;
               end loop;
               Ok := True;
               if not Any_Clamped or else Round = 3 then
                  return;
               end if;
            end;
         end;
      end loop;
   end Solve;

   function Row_Scale (E : Effect; Notch : Vec; R : Natural) return Long_Float is
      M : Long_Float := 0.0;
   begin
      for C in 0 .. E.N - 1 loop
         M := Long_Float'Max (M, abs (E.B (C, R)) * abs Notch (C));
      end loop;
      return M;
   end Row_Scale;

   procedure Solve_Priority (Hard, Soft : Term_Vectors.Vector; N : Natural; Cap : Vec; Active : Mask; Damp : Vec;
                             A : out Vec; Ok : out Boolean) is
      A1 : Vec := Zero_Vec;
      P : array (Ch_Index, Ch_Index) of Long_Float := [others => [others => 0.0]];
      Q : array (0 .. Rows * 8 - 1, Ch_Index) of Long_Float := [others => [others => 0.0]];
      NQ : Natural := 0;
   begin
      A := Zero_Vec;
      if Natural (Hard.Length) = 0 then
         Solve (Soft, N, Cap, Active, Damp, A, Ok);
         return;
      end if;
      Solve (Hard, N, Cap, Active, Damp, A1, Ok);
      if not Ok then
         return;
      end if;
      if Natural (Soft.Length) = 0 then
         A := A1;
         return;
      end if;
      --  硬约束那些行,正交化成 Q
      for T in 0 .. Natural (Hard.Length) - 1 loop
         for R in 0 .. Rows - 1 loop
            if Hard (T).W (R) > 0.0 and then NQ <= Q'Last (1) then
               declare
                  V : array (Ch_Index) of Long_Float := [others => 0.0];
                  Nm : Long_Float := 0.0;
               begin
                  for C in 0 .. N - 1 loop
                     V (C) := (if Active (C) then Hard (T).E.B (C, R) else 0.0);
                  end loop;
                  for K in 0 .. NQ - 1 loop
                     declare
                        D : Long_Float := 0.0;
                     begin
                        for C in 0 .. N - 1 loop
                           D := D + V (C) * Q (K, C);
                        end loop;
                        for C in 0 .. N - 1 loop
                           V (C) := V (C) - D * Q (K, C);
                        end loop;
                     end;
                  end loop;
                  for C in 0 .. N - 1 loop
                     Nm := Nm + V (C) * V (C);
                  end loop;
                  Nm := Sqrt (Nm);
                  if Nm > 0.0 then
                     for C in 0 .. N - 1 loop
                        Q (NQ, C) := V (C) / Nm;
                     end loop;
                     NQ := NQ + 1;
                  end if;
               end;
            end if;
         end loop;
      end loop;
      --  投影阵 P = I − QᵀQ
      for I in 0 .. N - 1 loop
         P (I, I) := 1.0;
      end loop;
      for K in 0 .. NQ - 1 loop
         for I in 0 .. N - 1 loop
            for J in 0 .. N - 1 loop
               P (I, J) := P (I, J) - Q (K, I) * Q (K, J);
            end loop;
         end loop;
      end loop;
      --  Soft 的雅可比右乘 P:在这套坐标下解出的 z,乘回 P 一定落在零空间里
      declare
         Sp : Term_Vectors.Vector;
         Z : Vec := Zero_Vec;
         Ok2 : Boolean;
         Worst : Long_Float := 1.0;
      begin
         for T in 0 .. Natural (Soft.Length) - 1 loop
            declare
               X : Term := Soft (T);
               Bp : Mat3 := [others => [others => 0.0]];
            begin
               for C in 0 .. N - 1 loop
                  for R in 0 .. Rows - 1 loop
                     declare
                        Acc : Long_Float := 0.0;
                     begin
                        for J in 0 .. N - 1 loop
                           Acc := Acc + Soft (T).E.B (J, R) * P (J, C);
                        end loop;
                        Bp (C, R) := Acc;
                     end;
                  end loop;
               end loop;
               --  误差要扣掉第一段已经走掉的那一部分
               declare
                  Got : constant Vec3 := Predict (Soft (T).E, A1);
               begin
                  for R in 0 .. Rows - 1 loop
                     X.Err (R) := Soft (T).Err (R) - Got (R);
                  end loop;
               end;
               X.E.B := Bp;
               Sp.Append (X);
            end;
         end loop;
         Solve (Sp, N, Cap, Active, Damp, Z, Ok2);
         if not Ok2 then
            A := A1;
            return;
         end if;
         for C in 0 .. N - 1 loop
            declare
               Acc : Long_Float := 0.0;
            begin
               for J in 0 .. N - 1 loop
                  Acc := Acc + P (C, J) * Z (J);
               end loop;
               A (C) := A1 (C) + Acc;
            end;
         end loop;
         --  越界只缩零空间那一半:硬约束已经达成的部分一点不动
         for C in 0 .. N - 1 loop
            if Cap (C) > 0.0 and then abs A (C) > Cap (C) then
               declare
                  Extra : constant Long_Float := abs (A (C) - A1 (C));
                  Room : constant Long_Float := Long_Float'Max (0.0, Cap (C) - abs A1 (C));
               begin
                  if Extra > 0.0 then
                     Worst := Long_Float'Min (Worst, Room / Extra);
                  else
                     Worst := 0.0;
                  end if;
               end;
            end if;
         end loop;
         if Worst < 1.0 then
            for C in 0 .. N - 1 loop
               A (C) := A1 (C) + (A (C) - A1 (C)) * Worst;
            end loop;
         end if;
      end;
      Ok := True;
   end Solve_Priority;

end Table;
